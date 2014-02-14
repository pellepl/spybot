import java.io.PrintStream;


public class MotorHolder {

  static double workPieceDepth;
  static double routerDia;
  static double motorWidth = 12+1.5;
  static double motorLength= 9+15+1.5;
  static double motorDepth = 10;
  static double flange = 10;
  static double wall = 2;
  static double feed = 150;
  static double zStep = 1;
  static double routeOverlap = 0.5;
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
      workPieceDepth = 13;
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
    out.println("(header)");
    out.println("G21");
    out.println("G90");
    out.println("G00 Z1");
    out.println("G00 X0 Y0");
    
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
        workPieceDepth + 0.7);
    
    out.println();
    out.println("(motor non flange right)");
    routeBox(
        +(wall/2 + (motorWidth + flange) / 2), motorLength / 4 + wall/2,
        flange - wall - routerDia, 
        motorLength / 2, 
        workPieceDepth - (motorDepth + wall),
        workPieceDepth + 0.7);

    out.println();
    out.println("(cutout)");
    cutout(0,0,
        motorWidth + flange * 2 + routerDia, 
        motorLength + wall * 2 + routerDia, 
        0,
        workPieceDepth + 0.7);
    
    out.println("(end)");
    out.println("G01 Z1");
    out.println("G00 X0 Y0");
    out.println("M05");
    out.println("M02");
  }
  
  // route a cutout from top to given z
  // does not take route diameter in account
  static void cutout(double cx, double cy, double cw, double ch, double startZ, double z) {
    if (startZ == z) return;
    System.out.println(
        "(cutout: middle " + cx + "," + cy + 
        " dim:" + cw + "x" + ch + 
        " from -" + startZ + " to -" + z + ")");
    
    double curZ = startZ + zStep;
    boolean stop = false;

    out.println("G00 Z1");
    out.println("G00 X" + (cx - cw/2) + " Y" +(cy - ch/2));
    
    do {
      out.println("G01 Z" + (-curZ) + " F" + feed);
      out.println("G01 X" + (cx + cw/2) + " Y" +(cy - ch/2));
      out.println("G01 X" + (cx + cw/2) + " Y" +(cy + ch/2));
      out.println("G01 X" + (cx - cw/2) + " Y" +(cy + ch/2));
      out.println("G01 X" + (cx - cw/2) + " Y" +(cy - ch/2));
      
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

    out.println("G00 Z1");

  }
  
  // route out a box from top to given z
  // does not take route diameter in account
  static void routeBox(double cx, double cy, double cw, double ch, double startZ, double z) {
    if (startZ == z) return;
    System.out.println(
        "(box: middle " + cx + "," + cy + 
        " dim:" + cw + "x" + ch + 
        " from -" + startZ + " to -" + z + ")");
    double curZ = startZ + zStep;
    boolean stop = false;
    
    out.println("G00 Z1");
    out.println("G00 X" + (cx - cw/2) + " Y" +(cy - ch/2));
    do {
      out.println("G01 Z" + (-curZ) + " F" + feed);
      routePlane(cx, cy, cw, ch);
      out.println("G01 Z" + (-(curZ-1)) + " F" + feed);
      out.println("G00 X" + (cx - cw/2) + " Y" +(cy - ch/2));
      
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

    out.println("G00 Z1");
  }
  
  // route out a plane starting at cx-cw/2,cy-cy/2
  // assuming z to be positioned, feed to be set, starting position to be set
  // does not take route diameter in account
  static void routePlane(double cx, double cy, double cw, double ch) {
    double routeStep = routerDia - routeOverlap;

    double y = cy-ch/2;
    boolean dir = true;
    boolean stop = false;
    do {
      if (dir) {
        out.println("G01 X" + (cx - cw/2) + " Y" + y);
        out.println("G01 X" + (cx + cw/2) + " Y" + y);
      } else {
        out.println("G01 X" + (cx + cw/2) + " Y" + y);
        out.println("G01 X" + (cx - cw/2) + " Y" + y);
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
  }
  
}
