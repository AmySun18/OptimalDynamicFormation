package coverage;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

class MinRobotPair{
	double distance;
	int id_in;
	int id_non;
	int index_non;
}

public class ShortestPathNoConstraint {

    private final Comparator shortestDistanceComparator = new Comparator()
    {
        public int compare(Object left, Object right)
        {
            assert left instanceof Robot && right instanceof Robot:"invalid comparison"; return compare((Robot) left, (Robot) right);
        }

        private int compare(Robot left, Robot right)
        {
            double result = left.costToBase - right.costToBase;

       //     if(result == 0)
      //          return 1;
          //  else
            return (int) result;
        }
    };
    Set<Robot> openSet = new HashSet<Robot>(); //TODO:use TreeSet
    public static final double  Eelec = 1;//nominal should be about 100
     public static final double Eamp = 0.01; //nJ/bit/m^2 , for free space
     public static final double Eamp2 = 0.1; //for the blocked portion
     Simulation sim;
     public ShortestPathNoConstraint(Simulation s)
     {
         sim = s;
     }
     
     public synchronized void buildRoutingTree()
     {
        ArrayList<Robot> robotInTree = new ArrayList<Robot>();
        ArrayList<Robot> robotNotInTree = new ArrayList<Robot>();
        
     	//openSet.clear();
        sim.robotList.get(0).descendants.clear();
         
        robotInTree.add(sim.robotList.get(0));  
 	        for (int i = 1; i < sim.realTimeRobotNumber; i++)
 	        {
 	            sim.robotList.get(i).descendants.clear();
 	            robotNotInTree.add(sim.robotList.get(i));	           
 	        }
 	        
 	    while(robotInTree.size()<sim.realTimeRobotNumber)
 	    {
 	    	MinRobotPair temp = new MinRobotPair();
 	    	temp.distance = 100000; 
// 	    //	 System.out.println("robotInTree: ");
//	    	    for(int i=0; i < robotInTree.size();i++)
//	            {
//	    		   System.out.println(robotInTree.get(i).id);
//	    		   
//	            }
//	    	    System.out.println("robotNotInTree: ");
// 	    	    for(int i=0; i < robotNotInTree.size();i++)
// 	            {
// 	    		   System.out.println(robotNotInTree.get(i).id);
// 	    		   
// 	            }
 	    	
    	    for(int i=0; i < robotInTree.size();i++)
            {
            	
            	for(int j=0; j < robotNotInTree.size(); j++)
            	{
            		double dist=point2.Dist(robotInTree.get(i).position, robotNotInTree.get(j).position);
            		
            		if(dist<temp.distance)
            		{
            			temp.distance = dist;
            			temp.id_in = robotInTree.get(i).id;
            			temp.id_non = robotNotInTree.get(j).id;
            			temp.index_non = j;
            		}
            	}
            	
            }
 	    	 //   System.out.println("The minmum pair is"+ temp.id_in + " " + temp.id_non);
 	    	    robotInTree.add(sim.robotList.get(temp.id_non));
 	    	    robotNotInTree.remove(temp.index_non);
 	    	    sim.robotList.get(temp.id_in).descendants.add(sim.robotList.get(temp.id_non));
 	    	    sim.robotList.get(temp.id_non).predecessor = sim.robotList.get(temp.id_in);
 	    	   // For debug
// 	    	    System.out.println("robotInTree: ");
// 	    	    for(int i=0; i < robotInTree.size();i++)
// 	            {
// 	    		   System.out.println(robotInTree.get(i).id);
// 	    		   
// 	            }
 	        
 	    }
 	    
 	    for(int i=0; i < sim.realTimeRobotNumber; i++)
 	    {
	    		System.out.println("descendants of node "+ i + " is");
	    		
 	    	for(int j=0; j< sim.robotList.get(i).descendants.size(); j++)
 	    	{
 	    		System.out.println(sim.robotList.get(i).descendants.get(j).id);
 	    	}
 	    }
 	      
     }
     
     public double link_cost_no_range_constraint(point2 p1, point2 p2)
     {
     	double points_dist = point2.Dist(p1, p2);
     	
 	    	 for (int i1 = 0; (i1 <  sim.obstacles.size()); i1++)
 	         {
 	             if (!(sim.obstacles.get(i1).LineOfSight(p1, p2)))
 	             {
 	            	 return -1;
 	             }
 	         }
 	    	 return points_dist;
     }
     
     void extractMinimum()
     {

         Iterator<Robot> i = openSet.iterator();


         Robot closestRobot = (Robot) i.next();
   //       Robot closestRobot = openSet.first();
         Robot tempRobot;
         while(i.hasNext())
         {
              tempRobot=    (Robot) i.next();
              if(tempRobot.costToBase<closestRobot.costToBase)
              {
                  closestRobot=tempRobot;
              }
         }

         openSet.remove(closestRobot);
         relaxNeighbors(closestRobot);
         closestRobot.predecessor.descendants.add(closestRobot);
     }
     void relaxNeighbors(Robot newlyRemoved)
     {
     	double new_cost_to_base;
     	Robot rbt;
     	
         for (Iterator<Robot> i = openSet.iterator(); i.hasNext(); )
         {
             rbt = (Robot) i.next();
             new_cost_to_base = newlyRemoved.costToBase + link_cost_no_range_constraint(rbt.position, newlyRemoved.position);
             if (rbt.costToBase > new_cost_to_base)
             {
                 // assign new shortest distance and mark unsettled
                 rbt.costToBase = new_cost_to_base;

                 // assign predecessor in shortest path
                 rbt.predecessor = newlyRemoved;
             }
         }
     }

     
        
}
