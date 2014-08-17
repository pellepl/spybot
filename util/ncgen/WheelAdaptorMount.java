import java.io.PrintStream;
import java.util.ArrayList;

public class WheelAdaptorMount {

  static double workPieceDepth = 11.5;
  static double routerDia = 2.8;
  static double feed = 100;
  static double zStep = 1.5;
  static double routeOverlap = 0.5;
  static double hoverDistance = 1.0;
  static double extraToBottom = 0.8;
  
  
  //    ________________               
  //   /               /|  drill: > 3mm           
  //  /_______________/ |  bottom to drill dist: 11mm             
  //  |      ___      | |  width: 25mm             
  //  |     /   \     | |  hole dia: 9.9mm              
  //  |     | O |     | |  hole depth: 4mm             
  //  |     \___/     | /  height: 22mm             
  //  |_______w________/                
  //                                      
  
  static double drillDia = 3.3;
  static double holeDia = 9.3;
  static double holeDepth = 4;
  static double holeCenterFromBottom = 11;
  static double width = 25;
  static double height = holeCenterFromBottom*2;
  
  static PrintStream out = System.out;

  /**
   * @param args
   */
  public static void main(String[] args) {

    workPieceDepth = 11.5;
    routerDia = 2.8;
    
    out.println("(work piece depth: " + workPieceDepth + ")");
    out.println("(router diameter:  " + routerDia + ")");
    out.println("(feed:             " + feed + ")");
    out.println("(z step:           " + zStep + ")");
    out.println();
    
    start();
    routeHole(0, 0, holeDia/2 - routerDia/2, 0, holeDepth);
    routeRing(0, 0, drillDia/2 - routerDia/2, 0, workPieceDepth+extraToBottom*2);

    ArrayList<Coord> cutOut = createPath();
    addToPath(cutOut, -width/2 - routerDia/2, 0);

    addToPath(cutOut, -width/2 - routerDia/2, -height/2 - routerDia/2 + 3);
    addToPath(cutOut, -width/2 - routerDia/2 + 3, -height/2 - routerDia/2);
    
    addToPath(cutOut, width/2 + routerDia/2 - 3, -height/2 - routerDia/2);
    addToPath(cutOut, width/2 + routerDia/2, -height/2 - routerDia/2 + 3);

    addToPath(cutOut, width/2 + routerDia/2, height/2 + routerDia/2 - 3);
    addToPath(cutOut, width/2 + routerDia/2 - 3, height/2 + routerDia/2);
    
    addToPath(cutOut, -width/2 - routerDia/2 + 3, height/2 + routerDia/2);
    addToPath(cutOut, -width/2 - routerDia/2, height/2 + routerDia/2 - 3);
    
    addToPath(cutOut, -width/2 - routerDia/2, 0);

    routePath(cutOut, 0, workPieceDepth+extraToBottom);
    
    end();
  }
  
  static ArrayList<Coord> createPath() {
    return new ArrayList<Coord>();
  }
  
  static ArrayList<Coord> addToPath(ArrayList<Coord> path, double x, double y) {
    path.add(new Coord(x,y));
    return path;
  }
  
  static void routePath(ArrayList<Coord> path, double startZ, double z) {
    if (startZ >= z) return;
    if (path.isEmpty()) return;
    out.println("(path)");
    double curZ = startZ + zStep;
    boolean stop = false;
    boolean dir = true;

    goUp();
    goXYRapid(path.get(0).x, path.get(0).y);
    
    do {
      goZ(curZ);
      for (Coord coord : path) {
        goXY(coord.x, coord.y);
      }
      goXY(path.get(0).x, path.get(0).y);
      
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
  static void routeCutoutRect(double cx, double cy, double cw, double ch, double startZ, double z) {
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
      goXYRapid(cx-cw/2, cy-ch/2, false);
      
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
  
  // route out a ring from start z to given z
  static void routeRing(double cx, double cy, double r, double startZ, double z) {
    if (startZ >= z) return;
    if (r <= 0) {
      throw new RuntimeException("Invalid radius (" + r + ")");
    }
    out.println(
        "(ring: center " + cx + "," + cy + 
        " radius:" + r + 
        " from -" + startZ + " to -" + z + ")");
    double curZ = startZ + zStep;
    boolean stop = false;
    
    goUp();
    goXYRapid(cx, cy-r);
    do {
      goZ(curZ);
     
      out.println("G02 X" + cx + " Y" + (cy+r) + " R" + r);
      out.println("G02 X" + cx + " Y" + (cy-r) + " R" + r);

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

    // up
    goUp();
  }

  // route out a hole from start z to given z
  static void routeHole(double cx, double cy, double r, double startZ, double z) {
    if (startZ >= z) return;
    if (r <= 0) {
      throw new RuntimeException("Invalid radius (" + r + ")");
    }
    out.println(
        "(hole: center " + cx + "," + cy + 
        " radius:" + r + 
        " from -" + startZ + " to -" + z + ")");
    double curZ = startZ + zStep;
    boolean stop = false;
    
    goUp();
    goXYRapid(cx, cy-r);
    do {
      goZ(curZ);
      double curR = r;
      while (curR > 0) {
        goXY(cx, cy-curR);
        out.println("G02 X" + cx + " Y" + (cy+curR) + " R" + curR);
        out.println("G02 X" + cx + " Y" + (cy-curR) + " R" + curR);
        curR -= (routerDia - routeOverlap); 
      }
      goXY(cx, cy-r);

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

    // up
    goUp();
  }
  
  // drill a hole from start z to given z
  static void routeDrill(double cx, double cy, double startZ, double z) {
    if (startZ >= z) return;
    out.println(
        "(drill: center " + cx + "," + cy + 
        " from -" + startZ + " to -" + z + ")");
    
    goUp();
    goXYRapid(cx, cy);
    goZ(startZ);
    goZ(z);

    // up
    goUp();
  }
  
  // peck drill a hole from start z to given z
  static void routeDrillPeck(double cx, double cy, double startZ, double z) {
    if (startZ >= z) return;
    out.println(
        "(drill peck: center " + cx + "," + cy + 
        " from -" + startZ + " to -" + z + ")");
    
    goUp();
    goXYRapid(cx, cy);
    double curZ = startZ;
    while (curZ < z) {
      curZ += zStep;
      if (curZ > z) curZ = z;
      goZ(curZ);
      goUp();
      if (curZ < z) goRapidDown(curZ - zStep);
    }

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

  static void goRapidDown(double z) {
    out.println("G00 Z" + (-z));
  }

  static void goXYRapid(double x, double y) {
    goXYRapid(x, y, true);
  }
  
  static void goXYRapid(double x, double y, boolean hover) {
    if (hover) out.println("G00 Z" + hoverDistance);
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
  
  static class Coord {
    double x; double y;
    public Coord(double x, double y) {
      this.x = x;
      this.y = y;
    }
  }
}
