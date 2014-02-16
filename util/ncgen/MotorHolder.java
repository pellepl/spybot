import java.io.PrintStream;

public class MotorHolder {

  static double workPieceDepth = 11.5;
  static double routerDia = 2.8;
  static double motorWidth = 12+0.5;
  static double motorLength= 9+15+0.5;
  static double motorDepth = 10;
  static double flange = 5;
  static double wall = 1.5;
  
  static double feed = 150;
  static double zStep = 1;
  static double routeOverlap = 0.5;
  static double hoverDistance = 1.0;
  static double extraToBottom = 0.8;
  
  static PrintStream out = System.out;

  // motor
  // 12x(9+15)x10
  //
  // holder (12+3*2+10x2)x(9+15+3*2)x(10+3)
  //
  //                      __                 __      
  //                     / /                / /|                                
  //                    / / |              / / |                                
  //                   / /  |             / /  |                              
  //           _______/ /   |  __________/ /___|__                             
  //          /        /    | /_________/        /|                            
  //         /        /     /|_________/        / |                             
  //        /   O    /     /          /    O   /  |                             
  //       /________/     /          /________/   |                             
  //       |        |    /           |        |   |                            
  //       |        |   /            |        |   |                               
  //       |        |  /_____________|        |   /                               
  //       |        | //____________/|        |  /                                
  //       |        |_|            |_|        | /                                 
  //       |__________________________________|/                                 
  
  /**
   * @param args
   */
  public static void main(String[] args) {

    if (args.length < 2) {
      out.println("(no depth or router diameter set)");
      out.println("(usage: java MotorHolder <work piece depth in mm> <router diameter in mm>)");
      workPieceDepth = 11.5;
      routerDia = 2.8;
      out.println("(assuming depth and diameter)");
    } else {
      workPieceDepth = Double.parseDouble(args[0]);
      routerDia = Double.parseDouble(args[1]);
    }
    
    out.println("(work piece depth: " + workPieceDepth + ")");
    out.println("(router diameter:  " + routerDia + ")");
    out.println("(motor depth:      " + motorDepth + ")");
    out.println("(motor width:      " + motorWidth + ")");
    out.println("(motor length:     " + motorLength + ")");
    out.println("(flange:           " + flange + ")");
    out.println("(wall:             " + wall + ")");
    out.println("(feed:             " + feed + ")");
    out.println("(z step:           " + zStep + ")");
    out.println("(final dimension:  "
        + (motorWidth + flange*2) + "x"
        + (motorLength + wall*2) + "x" +
        + (motorDepth + wall) + ")");
  
    out.println();
    start();
    
    out.println();
    out.println("(top shave)");
    routeBox(
        0, 0,
        motorWidth + flange * 2 + routerDia, 
        motorLength + wall * 2 + routerDia, 
        0,
        workPieceDepth - (motorDepth + wall));
    
    out.println();
    out.println("(motor fit + length wall)");
    routeBox(
        0, 0,
        motorWidth - routerDia, 
        motorLength + wall * 2 - routerDia, 
        workPieceDepth - (motorDepth + wall),
        workPieceDepth - (motorDepth + wall) + motorDepth - wall);
    
    out.println();
    out.println("(motor fit bottom)");
    routeBox(
        0, 0,
        motorWidth - routerDia, 
        motorLength - routerDia, 
        workPieceDepth - (motorDepth + wall) + motorDepth - wall,
        workPieceDepth - (motorDepth + wall) + motorDepth);
    
    out.println();
    out.println("(motor non flange left)");
    routeBox(
        -(wall/2 + (motorWidth + flange) / 2), motorLength / 4 + wall/2,
        flange - wall - routerDia, 
        motorLength / 2, 
        workPieceDepth - (motorDepth + wall),
        workPieceDepth + extraToBottom);
    
    out.println();
    out.println("(motor non flange right)");
    routeBox(
        +(wall/2 + (motorWidth + flange) / 2), motorLength / 4 + wall/2,
        flange - wall - routerDia, 
        motorLength / 2, 
        workPieceDepth - (motorDepth + wall),
        workPieceDepth + extraToBottom);

    out.println();
    out.println("(cutout)");
    routeCutout(0,0,
        motorWidth + flange * 2 + routerDia - 0.5, 
        motorLength + wall * 2 + routerDia - 0.5, 
        0,
        workPieceDepth + extraToBottom);
    
    end();
  }
  
  // route a groove defined from (x1,y1) to (x2,y2) from start z to given z
  static void routeGroove(double x1, double y1, double x2, double y2, double startZ, double z) {
    if (startZ >= z) return;
    out.println(
        "(groove: " + x1 + "," + y1 + 
        " to " + x2 + "," + y2 + 
        ", from -" + startZ + " to -" + z + ")");
    
    double curZ = startZ + zStep;
    boolean stop = false;
    boolean dir = true;

    goUp();
    goXYRapid(x1, y1);
    
    do {
      goZ(curZ);
      if (dir) {
        goXY(x1,y1);
        goXY(x2,y2);
      } else {
        goXY(x2,y2);
        goXY(x1,y1);
      }
      
      dir = !dir;
      
      if (curZ == z) {
        break;
      } else if (curZ + zStep > z) {
        curZ = z;
        if (stop) {
          break;
        } else {
          stop = true;
        }
      } else {
        curZ += zStep;
      }
    } while (true);

    goUp();
  }
  
  // route a cutout from start z to given z
  static void routeCutout(double cx, double cy, double cw, double ch, double startZ, double z) {
    if (startZ >= z) return;
    if (cw <= 0 || ch <= 0) {
      throw new RuntimeException("Invalid dimension (" + cw + "x" + ch + ")");
    }
    out.println(
        "(cutout: middle " + cx + "," + cy + 
        " dim:" + cw + "x" + ch + 
        " from -" + startZ + " to -" + z + ")");
    
    double curZ = startZ + zStep;
    boolean stop = false;

    goUp();
    goXYRapid(cx-cw/2, cy-ch/2);
    
    do {
      goZ(curZ);
      goXY(cx+cw/2, cy-ch/2);
      goXY(cx+cw/2, cy+ch/2);
      goXY(cx-cw/2, cy+ch/2);
      goXY(cx-cw/2, cy-ch/2);
      
      if (curZ == z) {
        break;
      } else if (curZ + zStep > z) {
        curZ = z;
        if (stop) {
          break;
        } else {
          stop = true;
        }
      } else {
        curZ += zStep;
      }
    } while (true);

    goUp();

  }
  
  // route out a box from start z to given z
  static void routeBox(double cx, double cy, double cw, double ch, double startZ, double z) {
    if (startZ >= z) return;
    if (cw <= 0 || ch <= 0) {
      throw new RuntimeException("Invalid dimension (" + cw + "x" + ch + ")");
    }

    out.println(
        "(box: middle " + cx + "," + cy + 
        " dim:" + cw + "x" + ch + 
        " from -" + startZ + " to -" + z + ")");
    double curZ = startZ + zStep;
    boolean stop = false;
    boolean planeDir = false;
    
    goUp();
    goXYRapid(cx-cw/2, cy-ch/2);
    do {
      goZ(curZ);
      routePlane(cx, cy, cw, ch, planeDir);
      goZ(curZ-hoverDistance);
      goXYRapid(cx-cw/2, cy-ch/2);
      
      planeDir = !planeDir;
      
      if (curZ == z) {
        break;
      } else if (curZ + zStep > z) {
        curZ = z;
        if (stop) {
          break;
        } else {
          stop = true;
        }
      } else {
        curZ += zStep;
      }
    } while (true);
    
    
    // finally route deep outer rectangle
    goZ(z);
    goXY(cx-cw/2, cy-ch/2);
    goXY(cx-cw/2, cy+ch/2);
    goXY(cx+cw/2, cy+ch/2);
    goXY(cx+cw/2, cy-ch/2);
    goXY(cx-cw/2, cy-ch/2);
    
    // up
    goUp();
  }
  
  // route out a plane starting at cx-cw/2,cy-ch/2
  // assuming z to be positioned, feed to be set, starting position to be set
  static void routePlane(double cx, double cy, double cw, double ch, boolean horiz) {
    if (cw <= 0 || ch <= 0) {
      throw new RuntimeException("Invalid dimension (" + cw + "x" + ch + ")");
    }

    double routeStep = routerDia - routeOverlap;
    if (horiz) {
  
      double y = cy-ch/2;
      boolean dir = true;
      boolean stop = false;
      do {
        if (dir) {
          goXY(cx-cw/2, y);
          goXY(cx+cw/2, y);
        } else {
          goXY(cx+cw/2, y);
          goXY(cx-cw/2, y);
        }
        dir = !dir;
        
        if (y == cy+ch/2) {
          break;
        } else if (y + routeStep > cy+ch/2) {
          y = cy+ch/2;
          if (stop) {
            break;
          } else {
            stop = true;
          }
        } else {
          y += routeStep;
        }
      } while (true);
      
    } else {
      
      double x = cx-cw/2;
      boolean dir = true;
      boolean stop = false;
      do {
        if (dir) {
          goXY(x, cy-ch/2);
          goXY(x, cy+ch/2);
        } else {
          goXY(x, cy+ch/2);
          goXY(x, cy-ch/2);
        }
        dir = !dir;
        
        if (x == cx+cw/2) {
          break;
        } else if (x + routeStep > cx+cw/2) {
          x = cx+cw/2;
          if (stop) {
            break;
          } else {
            stop = true;
          }
        } else {
          x += routeStep;
        }
      } while (true);
      
    }
  }
  
  static void goXY(double x, double y) {
    out.println("G01 X" + x + " Y" + y);
  }

  static void goZ(double z) {
    out.println("G01 Z" + (-z) + " F" + feed);
  }
  
  static void goUp() {
    out.println("G00 Z" + hoverDistance);
  }

  static void goXYRapid(double x, double y) {
    out.println("G00 Z" + hoverDistance);
    out.println("G00 X" + x + " Y" + y);
  }
  
  static void start() {
    out.println("(header)");
    out.println("G21");
    out.println("G90");
    out.println("G00 Z"+ hoverDistance);
    out.println("G00 X0 Y0");
  }

  static void end() {
    out.println("(end)");
    out.println("G01 Z" + hoverDistance);
    out.println("G00 X0 Y0");
    out.println("M05");
    out.println("M02");
  }
}
