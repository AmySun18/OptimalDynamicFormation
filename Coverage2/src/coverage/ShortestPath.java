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

public class ShortestPath
{
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

   // SortedSet openSet = new TreeSet(shortestDistanceComparator);
   Set<Robot> openSet = new HashSet<Robot>(); //TODO:use TreeSet
   public static final double  Eelec = 1;//nominal should be about 100
    public static final double Eamp = 0.01; //nJ/bit/m^2 , for free space
    public static final double Eamp2 = 0.1; //for the blocked portion
    Simulation sim;
    public ShortestPath(Simulation s)
    {
        sim = s;
    }
    
    public synchronized void build_potential_routing_tree()
    {
    	openSet.clear();
    	 sim.robotList.get(0).potential_descendants.clear();

         	 for (int i = 1; i < sim.realTimeRobotNumber; i++)
              {
                  sim.robotList.get(i).potential_descendants.clear();
                  sim.robotList.get(i).potential_predecessor = null;
                  
                  if(sim.robotList.get(i).is_seeker_mode)
                  {
                	  continue;
                  }
                  
                  openSet.add(sim.robotList.get(i));
                  sim.robotList.get(i).potential_costToBase = linkCost(sim.robotList.get(i).position, sim.robotList.get(0).position);
                  if(sim.robotList.get(i).potential_costToBase>=0)
                  {                	
                  	sim.robotList.get(i).potential_predecessor = sim.robotList.get(0);
                  }
              }

         	synchronized(openSet)
          	{
	              while (!openSet.isEmpty())
	              {
	              	boolean is_extract_success;
	              	
	              	is_extract_success =extractMinimum_constrained_potential();
	              
	              	if(!is_extract_success)
	              	{
	              		break;
	              	}
	              }
          	}
    }
    
    synchronized boolean extractMinimum_constrained_potential()
    {
    	Iterator<Robot> i = openSet.iterator();
        Robot closestRobot = (Robot) i.next();
        Robot tempRobot;
        while(i.hasNext())
        {
             tempRobot=(Robot) i.next();
             if(tempRobot.potential_costToBase>=0)
             {
            	 if ((closestRobot.potential_costToBase<0)||
            			 (closestRobot.potential_costToBase>=0&& tempRobot.potential_costToBase<closestRobot.potential_costToBase))
            	 {
	                 closestRobot=tempRobot;
	             } 
             }  
        }
        
        if(closestRobot.potential_costToBase<0)
        {
        	return false;
        }
        else
        {
	        openSet.remove(closestRobot);
	        relaxNeighbors_constrained_potential(closestRobot);
	        closestRobot.potential_predecessor.potential_descendants.add(closestRobot);
	        return true;
        }
    }
    
    synchronized void relaxNeighbors_constrained_potential(Robot newlyRemoved)
    {
    	double new_cost_to_base;
    	Robot rbt;
    	double new_link_cost;
    	
        for (Iterator<Robot> i = openSet.iterator(); i.hasNext(); )
        {
            rbt = (Robot) i.next();
            new_link_cost = linkCost(rbt.position, newlyRemoved.position);
            if(new_link_cost>=0)
            {
            	new_cost_to_base = newlyRemoved.potential_costToBase + linkCost(rbt.position, newlyRemoved.position);
            	 if ((rbt.potential_costToBase<0) || (rbt.potential_costToBase>=0 && rbt.potential_costToBase > new_cost_to_base))
                 {
                     // assign new shortest distance and mark unsettled
                     rbt.potential_costToBase = new_cost_to_base;

                     // assign predecessor in shortest path
                     rbt.potential_predecessor = newlyRemoved;
                 }
            }
            else
            {
            	continue;
            }         
        }
    }


    public synchronized void buildRoutingTree()
    {
    	openSet.clear();
        sim.robotList.get(0).descendants.clear();
        
        if(sim.coverage.has_connectivity_constraint)
        {
        	 for (int i = 1; i < sim.realTimeRobotNumber; i++)
             {
                 sim.robotList.get(i).descendants.clear();
                 sim.robotList.get(i).predecessor = null;
                 
                 if(sim.robotList.get(i).is_seeker_mode)
                 {
               	  continue;
                 }
                 
                 openSet.add(sim.robotList.get(i));
                 sim.robotList.get(i).costToBase = linkCost(sim.robotList.get(i).position, sim.robotList.get(0).position);
                 if(sim.robotList.get(i).costToBase>=0)
                 {                	
                 	sim.robotList.get(i).predecessor = sim.robotList.get(0);
                 }
             }

        	 synchronized(openSet)
           	{
	             while (!openSet.isEmpty())
	             {
	             	boolean is_extract_success;
	             	
	             	is_extract_success =extractMinimum_constrained();
	              	
	             	if(!is_extract_success)
	             	{
	             		break;
	             	}
	             }
           	}
        }
        else
        {
	        for (int i = 1; i < sim.realTimeRobotNumber; i++)
	        {
	            sim.robotList.get(i).descendants.clear();
	            openSet.add(sim.robotList.get(i));
	            sim.robotList.get(i).costToBase = linkCost(sim.robotList.get(i).position, sim.robotList.get(0).position);
	            sim.robotList.get(i).predecessor = sim.robotList.get(0);
	        }
	
	        synchronized(openSet)
          	{
		        while (!openSet.isEmpty())
		        {
		            extractMinimum();
		        }
          	}
        }
    }
    
    synchronized boolean extractMinimum_constrained()
    {
    	Iterator<Robot> i = openSet.iterator();
        Robot closestRobot = (Robot) i.next();
        Robot tempRobot;
        while(i.hasNext())
        {
             tempRobot=    (Robot) i.next();
             if(tempRobot.costToBase>=0)
             {
            	 if ((closestRobot.costToBase<0)||
            			 (closestRobot.costToBase>=0&& tempRobot.costToBase<closestRobot.costToBase))
            	 {
	                 closestRobot=tempRobot;
	             } 
             }  
        }
        
        if(closestRobot.costToBase<0)
        {
        	return false;
        }
        else
        {
	        openSet.remove(closestRobot);
	        relaxNeighbors_constrained(closestRobot);
	        closestRobot.predecessor.descendants.add(closestRobot);
	        return true;
        }
    }
    
    synchronized void relaxNeighbors_constrained(Robot newlyRemoved)
    {
    	double new_cost_to_base;
    	Robot rbt;
    	double new_link_cost;
    	
        for (Iterator<Robot> i = openSet.iterator(); i.hasNext(); )
        {
            rbt = (Robot) i.next();
            new_link_cost = linkCost(rbt.position, newlyRemoved.position);
            if(new_link_cost>=0)
            {
            	new_cost_to_base = newlyRemoved.costToBase + linkCost(rbt.position, newlyRemoved.position);
            	 if ((rbt.costToBase<0) || (rbt.costToBase>=0 && rbt.costToBase > new_cost_to_base))
                 {
                     // assign new shortest distance and mark unsettled
                     rbt.costToBase = new_cost_to_base;

                     // assign predecessor in shortest path
                     rbt.predecessor = newlyRemoved;
                 }
            }
            else
            {
            	continue;
            }
            
           
        }
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
            new_cost_to_base = newlyRemoved.costToBase + linkCost(rbt.position, newlyRemoved.position);
            if (rbt.costToBase > new_cost_to_base)
            {
                // assign new shortest distance and mark unsettled
                rbt.costToBase = new_cost_to_base;

                // assign predecessor in shortest path
                rbt.predecessor = newlyRemoved;
            }
        }
    }

    private double linkCost(point2 p1, point2 p2)
    {
        //TODO: use a table like hashmap to save calculated value in one iteration
    	double points_dist = point2.Dist(p1, p2);
    	
    	 if(sim.coverage.has_connectivity_constraint)
    	 {
	    	if(points_dist>sim.communication_range)
	    	{
	    		return -1; //indicate link infeasible    		
	    	}
	    	
	    	 for (int i1 = 0; (i1 <  sim.obstacles.size()); i1++)
	         {
	             if (!(sim.obstacles.get(i1).LineOfSight(p1, p2)))
	             {
	            	 return -1;
	             }
	         }
    	 }
    	
        double blockDist = 0;

        //does not consider the thickness of the blocking
      //  if (!sim.boundary.LineOfSight(p1, p2))
        {
       //     blockPenalty = 9999;
        }

        for (int i = 0; i < sim.obstacles.size(); i++)
        {
              blockDist+= sim.obstacles.get(i).BlockDistance(p1, p2);
        }

        //TODO IMPORTANT use the commented line for more realistic link cost, link_cost_gradient_x is designed for that case
        //return Eamp*Math.pow(points_dist, 2) + Eelec + Eamp2*Math.pow(blockDist,4);
        return points_dist;
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
   	 
   	
       double blockDist = 0;

       //does not consider the thickness of the blocking
     //  if (!sim.boundary.LineOfSight(p1, p2))
       {
      //     blockPenalty = 9999;
       }

       for (int i = 0; i < sim.obstacles.size(); i++)
       {
             blockDist+= sim.obstacles.get(i).BlockDistance(p1, p2);
       }

       return points_dist;
     //TODO IMPORTANT use the commented line for more realistic link cost
       //return Eamp*Math.pow(points_dist, 2) + Eelec + Eamp2*Math.pow(blockDist,4);
    }
    
    public double link_cost_gradient_x(double zi, point2 position, Robot predecessor, double obstacleBlockingDistance, point2 obstacleBlockingGradient)
    {
    	return
    	2 * ShortestPath.Eamp * zi * (position.x - predecessor.position.x) +
        4 * ShortestPath.Eamp2 * zi * Math.pow(obstacleBlockingDistance, 3) * obstacleBlockingGradient.x;
    }
    
    public double link_cost_gradient_y(double zi, point2 position, Robot predecessor, double obstacleBlockingDistance, point2 obstacleBlockingGradient)
    {
    	return
    	2 * ShortestPath.Eamp * zi * (position.y - predecessor.position.y) +
        4 * ShortestPath.Eamp2 * zi * Math.pow(obstacleBlockingDistance, 3) * obstacleBlockingGradient.y;
    }

}
