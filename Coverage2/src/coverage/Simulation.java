package coverage;

import java.awt.Toolkit;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;
import java.util.SortedSet;


import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import javax.swing.SwingWorker;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class Simulation
{
    public ArrayList<Robot> robotList;
    Obstacle boundary;
    ArrayList<Obstacle> obstacles;

    double simulationTime = 0;  //global clock
    double timeStep = 0.01;     //increment of the simulation time
    public int realTimeRobotNumber;
    ArrayList<point2> robot_init_pos_list;
    Coverage coverage;
    public double weightedObjectiveValue=0;

    //Coverage evaluation
    public  double objEvalIncrement = 1; //this increment was 0.1.  0.25, 0.5 or 1 might be good. If set to 0.75, there is size problem with coverageMap
    public  double[][] coverageMap;

    //print out control
    int printFrequencyCounter=0;

    //communication
//    public Robot baseRobot=new Robot(); //base station node; its location will be initilized later
    int routingUpdateCounter = 1;
    ShortestPath pathGenerator = new ShortestPath(this);
    ShortestPathNoConstraint path = new ShortestPathNoConstraint(this);
    public int broadcastCount = 0;
    public int requestCount = 0;
    public int totalbroadcastCount=0;
    public double communication_range = 10.0;
    public boolean RandomOrder = true;
    public boolean BreadthFirst = false;
    public boolean Projection = false;

    //scan stuff
     public  double[][] scanMap;
     public double scanIncrement = 1;
     private double scanValueMax=-1; //use to show make gradient color more obvious
     private double scanValueMin=9999999;
     public double scanValueDiff=0;
     public double scanValueBase =0;

     //static event related
      EventDensity density;       //gives the current guess of the true event density (which is hidden from robots) and generates events
      ArrayList<Event> currentDetectedEvents = new ArrayList<Event>();  //record the detected events still alive on the map
      boolean manualEventManagement = true;
      
      //moving target related 
      Target target1;  //TODO: expand this to a list targets
      private boolean targetAppears = false;
      
      //obstacle discovery stuff
      public static double rangefinderRange = 8;  //initial value
      
      //required by thread safety
      boolean to_reset = false;
      
      //when there is comm. range constraint, leaf relaxation stuff
      double leaf_relax_strength = 1.5;
      
      //map building stuff
      public int[][] mapBuildingGrid;  //0: unknown, 1: discovered, 2: inside obstacle, 3:frontier 
      public double mapBuildingIncrement = 1;
      boolean isAllSpaceDiscovered = false;
      
      //background task
      UpdateObjInfoTask updateObjInfoTask;

    public Simulation(Obstacle bndy, ArrayList<Obstacle> obstls, EventDensity dnst, int robot_num, 
    		double max_speed, ArrayList<point2> _robot_init_pos_list, double comm_range, Coverage cvrg, double sensing_decay)
    {
        robotList = new ArrayList<Robot>();
        boundary = bndy;
        obstacles = obstls;
        density = dnst;
        realTimeRobotNumber = robot_num;
        robot_init_pos_list = _robot_init_pos_list;
        coverage = cvrg;
        this.communication_range = comm_range;

        double _controlPeroid = 0.1; //0.1 is our standard
        double _width = 2;

//        baseRobot.position = new point2(0.01,0.01); //was 0.01, 0.01
//        baseRobot.estimatedPosition = new point2(0.01,0.01);  //was 0.01, 0.01
//        baseRobot.id = -1;
//        baseRobot.costToBase = 0;
//        baseRobot.predecessor = null;

        for (int i2 = 0; i2 < coverage.MAX_ROBOT_NUMBER; i2++)
        {        	
        	point2 new_robot_pos;
        	if(i2<robot_init_pos_list.size())
    		{
        		new_robot_pos = new point2(robot_init_pos_list.get(i2).x,robot_init_pos_list.get(i2).y);
    		}
        	else
        	{
        		new_robot_pos = new point2(0.1+i2 *0.5 , 0.1+i2 *0.5);
        	}
        	
        	if(coverage.is_obstacle_discovery_mode)
        	{
        		
        		robotList.add(new MapBuildingRobot(i2,new_robot_pos , 0.0, 0.0, 0.0, _width, max_speed, 0.0, _controlPeroid,
                     0.0 + i2 * timeStep, boundary, obstacles, density,this,sensing_decay));  		
        	}
        	else
        	{
        		robotList.add(new Robot(i2, new_robot_pos , 0.0, 0.0, 0.0, _width, max_speed, 0.0, _controlPeroid,
                              0.0 + i2 * timeStep, boundary, obstacles, density,this,sensing_decay));
        	}
          //  robotList.add(new Robot(i2, new point2(8.0 , 24.0+i2*2.0), 0.0, 0.0, 0.0, _width, _maxSpeed, 0.0, _sensingDecay, _controlPeroid,
            //                     0.0, boundary, obstacles, density,this));
           
        	// robotList.get(i2).predecessor = baseRobot;
        }
        
        if(this.coverage.is_link_cost_mode || this.coverage.has_connectivity_constraint)
        {
        	pathGenerator.buildRoutingTree();
        }
        
        coverageMap = new double[(int)(boundary.largestY/objEvalIncrement)][(int)(boundary.largestX/objEvalIncrement)];
        scanMap = new double[(int)(boundary.largestY/scanIncrement)][(int)(boundary.largestX/scanIncrement)];
        mapBuildingGrid = new int[(int)(boundary.largestX/mapBuildingIncrement)][(int)(boundary.largestY/mapBuildingIncrement)];
        
       //moving target related
        target1 = new Target();
       
    }

    //used for scanning the map for optimal deployment of next node
    public synchronized void scanWithOneAdditionalNode()
    {
        int scanMapRow = 0;
        int scanMapColumn = 0;

        scanValueBase=EvaluateObj(); //without the addtional node
        scanValueDiff=boundary.largestX*boundary.largestY-scanValueBase;
        realTimeRobotNumber++;

        for (double horizontalSamplePoint = 0.01; horizontalSamplePoint <= boundary.largestX; horizontalSamplePoint += scanIncrement)
            {
                for (double verticalSamplePoint = 0.01; verticalSamplePoint <= boundary.largestY; verticalSamplePoint += scanIncrement)
                {
                    boolean isInsideObstacle = false;
                    for (int i = 0; i < obstacles.size(); i++)
                    {

                        if ((obstacles.get(i).IsInteriorPoint(new point2(horizontalSamplePoint,verticalSamplePoint))))
                        {
                            isInsideObstacle = true;
                            break;
                        }
                    }

                    if(!isInsideObstacle)
                    {
                    robotList.get(realTimeRobotNumber-1).position.x=horizontalSamplePoint;
                    robotList.get(realTimeRobotNumber-1).position.y=verticalSamplePoint;
                    scanMap[scanMapRow][scanMapColumn]= EvaluateObj();//was using UpdateObjInfo();
                    if(scanMap[scanMapRow][scanMapColumn]>this.scanValueMax)
                    {
                        scanValueMax=scanMap[scanMapRow][scanMapColumn];
                    }
                    if(scanMap[scanMapRow][scanMapColumn]<this.scanValueMin)
                    {
                        scanValueMin=scanMap[scanMapRow][scanMapColumn];
                    }
                    }
                    else
                    {
                         scanMap[scanMapRow][scanMapColumn]=scanValueBase;
                    }
                    System.out.print(" "+(float)(scanMap[scanMapRow][scanMapColumn]));
                    scanMapRow++;
                }
                System.out.println();
                scanMapColumn++;
                scanMapRow=0;
            }
          scanValueBase=scanValueMin;
        scanValueDiff=scanValueMax-scanValueBase;
        realTimeRobotNumber--;
       scanValueMax=-1;
       scanValueMin=9999999;
    }

    public void simulateOneStep()
    {
    	if(to_reset)
    	{
    		this.reset_robots();
    		this.ResetObstacles();
    		to_reset = false;
    	}
    	for(int i=0; i < realTimeRobotNumber; i++)
    	{
    		robotList.get(i).InformationValue();
    	}
    	if(Projection)
    	{
    		// Create a Q, 0 is the first element and there are N+1 elements.
    		int[] Q = new int[realTimeRobotNumber]; 
    		Q=creatQueue();
    		Q=removeDupInIntArray(Q);
    		// Check connected or not
    		
    		for(int j=1;j<realTimeRobotNumber;j++)
    		{
    			// store old position
    			point2 oldPosition = new  point2(robotList.get(Q[j]).position.x, robotList.get(Q[j]).position.y);
    			// new candidate position
    			robotList.get(Q[j]).position. x = robotList.get(Q[j]).position. x + 1;
    			boolean needProjection = true;
    			for(int j1=0; j1<j; j1++)
    			{// j1 has updated its position
    				// Neighbor or not?
					if(point2.Dist(oldPosition, robotList.get(Q[j1]).position)>2*robotList.get(0).sensingCutoffRange)
					{
						continue;
					}
					else if(robotList.get(Q[j]).isPointInLOS(robotList.get(Q[j1]).position))
					{
						double distance_candidate = point2.Dist(robotList.get(Q[j]).position, robotList.get(Q[j1]).position);
						if(distance_candidate < communication_range)
						{
							needProjection = false;
							break;
						}
					}
					
					
    			}
    			
   			
    			if(needProjection)
    			{
    				// Projection candidate
    				// from the old position
    				robotList.get(Q[j]).position. x = robotList.get(Q[j]).position. x - 1;
    				point2[] ProjecCandidate = new point2[j];
    				double[] ProjecCandidateDistance = new double[j];
    				for(int j1=0; j1<j; j1++)
    				{
    					// Neighbor or not?
    					if(point2.Dist(robotList.get(Q[j]).position, robotList.get(Q[j1]).position)>2*robotList.get(0).sensingCutoffRange)
    					{
    						ProjecCandidateDistance[j1] = 1000;
    						continue;
    					}
    					// Visible or not?
    					else if(robotList.get(Q[j]).isPointInLOS(robotList.get(Q[j1]).position))
    	    			{ // keep connected with a updated node j1;
    	    				//robotList.get(Q[j]).predecessor = robotList.get(Q[j1]);
    						double distance_candidate = point2.Dist(robotList.get(Q[j]).position, robotList.get(Q[j1]).position);
    	    				ProjecCandidate[j1] = point2.plus(robotList.get(Q[j1]).position,
    	    						point2.divide( point2.minus(robotList.get(Q[j]).position, robotList.get(Q[j1]).position),
			        						distance_candidate/(0.9999*communication_range)));
    	    				ProjecCandidateDistance[j1] = point2.Dist(oldPosition, ProjecCandidate[j1]);
		        			
    	    			}
    					// Obstacle blocks
    					else 
    					{
    						BlockedObstacle  BlockObs= new BlockedObstacle(-1,-1);
    						BlockObs = robotList.get(Q[j]).blockedObs(robotList.get(Q[j1]).position);
    						// verticeBlockObs x-0.2 to escape the exactly the obstacle vertex, 
    						point2 verticeBlockObs = new point2(obstacles.get(BlockObs.obstacleNum).vertices.get(BlockObs.verticeNum).x-0.2, obstacles.get(BlockObs.obstacleNum).vertices.get(BlockObs.verticeNum).y);
//    						System.out.println("Obstacle "+BlockObs.obstacleNum +" blocks these two points." + 
//    						"vertice is "+ obstacles.get(BlockObs.obstacleNum).vertices.get(BlockObs.verticeNum).x + 
//    						" " +obstacles.get(BlockObs.obstacleNum).vertices.get(BlockObs.verticeNum).y );
//    						
    						double distance_candidate = point2.Dist(robotList.get(Q[j1]).position, verticeBlockObs);
    	    				ProjecCandidate[j1] = point2.plus(robotList.get(Q[j1]).position,
    	    						point2.divide( point2.minus(verticeBlockObs, robotList.get(Q[j1]).position),
			        						distance_candidate/(0.9999*communication_range)));
    	    				
    	    				ProjecCandidateDistance[j1] = point2.Dist(oldPosition, ProjecCandidate[j1]);
    						
    					}
    				}
    				// Select the candidate with the minimal distance
    				double mindis = 1000;
    				int mindisIndex = -1;
    				for (int j1=0; j1<j; j1++)
    				{
    					if(ProjecCandidateDistance[j1] <mindis)
    					{
    						mindis = ProjecCandidateDistance[j1];
    						mindisIndex = j1;
    					}
    				}
    				
    				robotList.get(Q[j]).position. x = ProjecCandidate[mindisIndex].x;
    				robotList.get(Q[j]).position. y = ProjecCandidate[mindisIndex].y;
    				
    			}
    				
    		}
    			
    		// If not, project
    		//skip robotList.updates
    		Projection=false;
    		//printInverseTree();
    	}
    	else
    	{
    		for (int i = 1; i < realTimeRobotNumber; i++)
    	    	
    		{
            robotList.get(i).UpdateStates(simulationTime);

            //Generate neighbors
            robotList.get(i).neighbors.clear();
            for (int i1 = 0; i1 < realTimeRobotNumber; i1++)
            {
                if (i1 != i)
                {
                    if (point2.Dist(robotList.get(i).position, robotList.get(i1).position) < robotList.get(i).sensingCutoffRange * 2) //SYNC
                    {
                        robotList.get(i).neighbors.add(robotList.get(i1));
                    }
                }
            }
         }

    		
    	}

        if(this.coverage.is_link_cost_mode || this.coverage.has_connectivity_constraint)
        {
        	pathGenerator.buildRoutingTree();  
        }
        //event management
        if(!manualEventManagement)
        {
            Event newEvent = new Event();
            if (density.generateEvent(newEvent, simulationTime))
            {
                // System.out.println("an event is generated at: "+newEvent.eventLocation.x+ ","+newEvent.eventLocation.y);

                //TODO: test if this event is detected
                //assuming all detected for now
                currentDetectedEvents.add(newEvent);
            }
            //delete dead event, assuming all events are added sequentially and with interval larger than the interval of this test.
            //so only the first event in the list needs to be tested for deletion
            if (currentDetectedEvents.size() > 0)
            {
                if (simulationTime - currentDetectedEvents.get(0).spawnTime > Event.EVENT_DURATION)
                {
                    //  System.out.println("an event is removed at: " + currentDetectedEvents.get(0).eventLocation.x + "," +
                    //                      currentDetectedEvents.get(0).eventLocation.y);
                    currentDetectedEvents.remove(0);
                }
            }
        }
        //event management ends

        //moving target management 
        if(targetAppears)
        {
        	target1.targetLocationUpdate(simulationTime);
       		this.density.peak.x = target1.x;
       		this.density.peak.y = target1.y;
       		this.density.peak_in_effect = target1.trajectoryStarted && !target1.trajectoryEnded;
        }
        //moving target management ends
        
        //moving data source management
        if(coverage.has_moving_data_source)
        {
        	density.UpdateDataSourceStates(simulationTime, obstacles,this.robotList,realTimeRobotNumber);
        }
        //moving data source management ends
        
        //TODO detect only when in FOV
        //detect obstacle vertices  
        if(this.coverage.is_obstacle_discovery_mode)
        {
        	for (int i = 0; i < obstacles.size(); i++)
	        {
	        	if(!obstacles.get(i).isAllVerticesDiscovered)
	        	{
		        	 for (int i1 = 0; i1 < obstacles.get(i).vertices.size(); i1++)
		        	 {
		        		if(!obstacles.get(i).vertices.get(i1).isDiscovered) 
		        		{
		        			 for (int i2 = 0; i2 < this.realTimeRobotNumber; i2++)
		    	        	 {	        				   				 
		        				 if(point2.Dist(this.robotList.get(i2).position,obstacles.get(i).vertices.get(i1))<rangefinderRange	)	 
		        				 {
		        					 point2 toPrevious; //a vector from current vertex to the previous one
		        					 point2 toNext;
		        					 if(i1!=0)
		        						 toPrevious = point2.minus( obstacles.get(i).vertices.get(i1-1),obstacles.get(i).vertices.get(i1));
		        					 else
		        						 toPrevious = point2.minus(obstacles.get(i).vertices.get(obstacles.get(i).vertices.size()-1),obstacles.get(i).vertices.get(i1));
		        							 
		        					 if ( i1 != obstacles.get(i).vertices.size()-1)
		        						 toNext = point2.minus(obstacles.get(i).vertices.get(i1+1),obstacles.get(i).vertices.get(i1));
		        					 else
		        						 toNext = point2.minus(obstacles.get(i).vertices.get(0),obstacles.get(i).vertices.get(i1));
		        						 
		        					 point2 toRobot = point2.minus(robotList.get(i2).position, obstacles.get(i).vertices.get(i1));
		        					 point2 toInterior = point2.minus(obstacles.get(i).interiorPoint, obstacles.get(i).vertices.get(i1));
	
		        					 if (point2.cross(toRobot, toPrevious )*point2.cross(toInterior, toPrevious)>0
		        						 && point2.cross(toRobot, toNext )*point2.cross(toInterior, toNext)>0 )
		        					 {
		    	        				 continue;
		        					 }
		        					 
		        					 boolean isVisible = true;
		        					 for (int j = 0; j < obstacles.size(); j++)
		        			         {
		        			             if (j!=i && !obstacles.get(j).LineOfSight(robotList.get(i2).position, obstacles.get(i).vertices.get(i1)))
		        			             {
		        			            	 isVisible = false;
		        			             }
		        			         }
		        					 
		        					 if (isVisible)
			        					 {
			        					 obstacles.get(i).vertices.get(i1).isDiscovered = true;
			        					 obstacles.get(i).discoveredVerticesNumber++;
			        					 if(obstacles.get(i).discoveredVerticesNumber==obstacles.get(i).vertices.size())
			        					 {
			        						 obstacles.get(i).isAllVerticesDiscovered = true;
			        						 obstacles.get(i).vertices = obstacles.get(i).originalVertices;
			        					 }
			        					 break;
		        					 }
		        				 }
		    	        	 }
		        		}
		        	 }
	        	}
	        }
        	
        	if(!isAllSpaceDiscovered)
        	{
    			isAllSpaceDiscovered = true; //it will be set to false if some unknown or frontier space is discovered
    			point2 testPoint = new point2();
    			
    			for(int i=0; i<mapBuildingGrid.length;i++)
            	{
            		for(int j=0; j<mapBuildingGrid[i].length;j++)
	        		{
	        			if(this.mapBuildingGrid[i][j]==0 ||this.mapBuildingGrid[i][j]==3)
	        			{	
	        				testPoint.x = (0.5+i)*this.mapBuildingIncrement;
	        				testPoint.y = (0.5+j)*this.mapBuildingIncrement;
	        				isAllSpaceDiscovered = false;
	        				        				
	        				//if it is detected by a robot
	        				for(int k = 0; k<this.realTimeRobotNumber;k++)
	        				{
	        					 if(point2.Dist(this.robotList.get(k).position,testPoint)<rangefinderRange-this.mapBuildingIncrement) //need -mapBuildingIncrement to make sure obstacle boundary is founded when all empty space is visited	 
		        				 {
	        						 boolean isVisible = true;
	        						 for (int k1 = 0; k1 < obstacles.size(); k1++)
		        			         {
		        			             if (!obstacles.get(k1).LineOfSight(robotList.get(k).position, testPoint))
		        			             {
		        			            	 isVisible = false;
		        			            	 break;
		        			             }
		        			         }
	        						 
	        						 if(isVisible)
	        						 {
	        							 this.mapBuildingGrid[i][j]=1;
	        							 break;
	        						 }
		        				 }
	        				}
	        				
	        				//if it is inside an obstacle
	        				for(int k = 0; k<this.obstacles.size();k++)
	        				{
	        					if(this.obstacles.get(k).IsInteriorPoint(testPoint))
	        					{
	        						this.mapBuildingGrid[i][j]=2;
	        						break;
	        					}
	        				}
	        			}
	        		}
	        	}
        	}

        	//find frontier 
        	for(int i=0; i<mapBuildingGrid.length;i++)
        	{
        		for(int j=0; j<mapBuildingGrid[i].length;j++)
        		{
        			if(this.mapBuildingGrid[i][j]==0 ||this.mapBuildingGrid[i][j]==3)
        			{
        				boolean isFrontier = false;
        				if(i>0 && this.mapBuildingGrid[i-1][j]==1) isFrontier = true;
        				if(j>0 && this.mapBuildingGrid[i][j-1]==1) isFrontier = true;
        				if(i<mapBuildingGrid[i].length-1 && this.mapBuildingGrid[i+1][j]==1) isFrontier = true;
        				if(j<mapBuildingGrid[i].length-1 && this.mapBuildingGrid[i][j+1]==1) isFrontier = true;
        				
        				if(isFrontier)
        				{
        					this.mapBuildingGrid[i][j]=3;
        				}
        				else
        				{
        					this.mapBuildingGrid[i][j]=0;
        				}
        			}
        		}
        	}
        }
        
        //detect obstacle vertices ends

        simulationTime += timeStep;
        

        
        //just for debuging
        //System.out.println("A: robot "+0 + "'s predecessor is "+ robotList.get(0).predecessor.id);
        //System.out.println("A: robot "+1 + "'s predecessor is "+ robotList.get(1).predecessor.id);
            
        if(routingUpdateCounter%10==0)
        {
            //borrow this counter for other purpose: print how many communication for motion control happened in this period
           //  System.out.println("broadcast:"+Integer.toString(broadcastCount)+'\t'+'\t'+"request:"+Integer.toString(requestCount));
             totalbroadcastCount+=broadcastCount;
             broadcastCount = 0;
             requestCount = 0;
            //borrow end          
        }
        routingUpdateCounter++;

        if (coverage.evaluateObjFunc)
        {
            UpdateObjInfo();
            //coverage.realTimeObjFuncDisplay.setText(Double.toString(EvaluateObjectiveFunc()));
            //coverage.weightedObjFuncDisplay.setText(Double.toString(weightedObjectiveValue));

            if(printFrequencyCounter%10==0)
            {
                //System.out.println(Double.toString(simulationTime)+'\t'+coverage.realTimeObjFuncDisplay.getText());
                //Also print out the communication count
//                System.out.println(Double.toString(simulationTime)+'\t'+coverage.realTimeObjFuncDisplay.getText()+'\t'+totalbroadcastCount);
            }
            printFrequencyCounter++;
        }
        
//   
        
		try {
			
			 
			String content = "This is the content to write into file";
			
			for (int i=0;i<realTimeRobotNumber;i++)
			{
 
				
				
				content =  content + "\n"+ "Id: " + Integer.toString(i) + " x: " + Double.toString(robotList.get(i).position.x)
						+" y: " + Double.toString(robotList.get(i).position.y);
				
			}
 
			File file = new File("/users/xmsun/Desktop/filename.txt");
			 
			// if file doesnt exists, then create it
			if (!file.exists()) {
				file.createNewFile();
			}
			FileWriter fw = new FileWriter(file.getAbsoluteFile());
			BufferedWriter bw = new BufferedWriter(fw);
			bw.write(content);
			bw.close();
			
 
			System.out.println("Done");
 
		} catch (IOException e) {
			e.printStackTrace();
		}

    }

    public void relax_leaf_nodes() //can only relax to the nodes on current path to base
    {
    	for (int i = 0; i < realTimeRobotNumber; i++)
        {
    		//System.out.println("node "+i+" has descendants:"+robotList.get(i).descendants.size());
    		if(this.robotList.get(i).descendants.isEmpty())
    		{
    			 System.out.println("node "+i+" is a leaf");
    			 Robot current_node = robotList.get(i).predecessor;
    			 double current_path_length = 0;
    			 if(current_node != null)
    			 {
    				 current_path_length = point2.Dist(current_node.position, robotList.get(i).position);
    			 }
    			    			 
    			 while(true)
    			 {
    				 if(current_path_length > 1.5* point2.Dist(current_node.position, robotList.get(i).position)) //1.5 is a design parameter, smaller values means more relaxation
    				 {
    					 System.out.println("node "+i+" is relaxed to node "+current_node.id);
    					 robotList.get(i).move_to_robot(current_node,this.communication_range);
    					 //robotList.get(i).position.x = current_node.position.x;
    					 //robotList.get(i).position.y = current_node.position.y;
    					 break;
    				 }
    				 
    				 if(current_node.predecessor == null)
    				 {
    					 break;
    				 }
    				 else
    				 {
    					 current_path_length += point2.Dist(current_node.position, current_node.predecessor.position);
    					 current_node = current_node.predecessor;
    				 }
    			 }
    		}
        }
    }
    
    //TODO: should use estimated position instead of position
    synchronized public void relax_leaf_nodes_alt() //look at all nodes for relaxation target
    {
    	for (int i = 0; i < realTimeRobotNumber; i++)
        {
    		//System.out.println("node "+i+" has descendants:"+robotList.get(i).descendants.size());
    		if(this.robotList.get(i).descendants.isEmpty()&&this.robotList.get(i).predecessor!=null)
    		{
    			 System.out.println("node "+i+" is a leaf");
    			 
    			 if(this.robotList.get(i).predecessor.id<0) //its predecessor is base station, id = -1
    			 {
    				 continue;
    			 }
    			 
    			 double closest_relax_target_distance = Double.MAX_VALUE;
    			 int closest_relax_target_id = -2; //-1 is base station, -2 means no relaxation target is found yet
    			 
    			 for (int i1 = 0; i1 < realTimeRobotNumber; i1++)
    			 {
    				 double distance_to_neighbor = point2.Dist(robotList.get(i).position, robotList.get(i1).position);
    				 double cost_to_neighbor = this.pathGenerator.link_cost_no_range_constraint(robotList.get(i).position, robotList.get(i1).position);
    				 
    				 if(this.robotList.get(i).costToBase > this.leaf_relax_strength*(this.robotList.get(i1).costToBase+cost_to_neighbor) 
    						 && distance_to_neighbor<closest_relax_target_distance)
    				 {
    					 closest_relax_target_distance =  distance_to_neighbor;
    					 closest_relax_target_id = i1;
    				 }
    			 }	
    			 
//    			 //base station -1 is another possibility
//    			 double distance_to_base = point2.Dist(robotList.get(i).position, this.baseRobot.position);
//				 double cost_to_base = this.pathGenerator.link_cost_no_range_constraint(robotList.get(i).position, baseRobot.position);
//				 
//				 if(this.robotList.get(i).costToBase > this.leaf_relax_strength*(cost_to_base) 
//						 && distance_to_base<closest_relax_target_distance)
//				 {
//					 System.out.println("node "+i+" is relaxed to base station");
//					 robotList.get(i).move_to_robot(this.baseRobot,this.communication_range/2);
//				 }
//				 else if(closest_relax_target_id>=0)
//    			 {
//    				 System.out.println("node "+i+" is relaxed to node "+closest_relax_target_id);
//					 robotList.get(i).move_to_robot(this.robotList.get(closest_relax_target_id),this.communication_range/2);
//    			 }
    		}
        }
    }
    
    public double EvaluateObj()
    {
    	double objective = 0;
        point2 samplePoint = new point2(-1,-1);
        double miss_prob;
         double alpha;
         try
         {
             for (double horizontalSamplePoint = 0.01; horizontalSamplePoint <= boundary.largestX;
                                                 horizontalSamplePoint += objEvalIncrement)
             {
                 for (double verticalSamplePoint = 0.01; verticalSamplePoint <= boundary.largestY; verticalSamplePoint += objEvalIncrement)
                 {
                     miss_prob = 1;
                     samplePoint.x = horizontalSamplePoint;
                     samplePoint.y = verticalSamplePoint;
                   
                        for (int i5 = 0; i5 < realTimeRobotNumber; i5++)
                         {
                             if (point2.Dist(robotList.get(i5).position, samplePoint) < robotList.get(i5).sensingCutoffRange) //SYNC
                             {
                                 if (boundary.LineOfSight(robotList.get(i5).position, samplePoint))
                                 {
                                     alpha = 1; //sensing ability discount factor
                                   
                                     if(!robotList.get(i5).is_point_visible(samplePoint))
                                     {
                                    	 alpha = EventDensity.WALL_DECAY_FACTOR;
                                     }                                   
                                     
                                     if(alpha>0)
                                     {
                                    	 miss_prob *= (1 - 
                                    	 alpha * robotList.get(i5).SensingModelFunction(point2.Dist(robotList.get(i5).position, samplePoint)));
                                     }
                                 }
                             }
                         }

                     objective += (density.GetEventDensity(samplePoint) * (1 - miss_prob));
                 }
             }
         }
         catch(ArrayIndexOutOfBoundsException e)
         {
        	 e.printStackTrace();
             return -1;
         }

         return objective * objEvalIncrement * objEvalIncrement;
    }
    
    void UpdateObjInfo()
    {
    	 //update obj info background
        updateObjInfoTask = new UpdateObjInfoTask(this);
    	updateObjInfoTask.execute();
    }
    
    synchronized void UpdateObjInfoActual()
    {
       // for (int i = 0; i < realTimeRobotNumber; i++)
       // {
            //DP2("POS=%f,%f\n",pRobotList[robotIterator]->position.x,pRobotList[robotIterator]->position.y);
            //System.out.print("POS="+robotList.get(i).position.x+","+robotList.get(i).position.y+"\n");
       // }

        //Evaluate the objective function based on the current robot positions
        double objective = 0;
        weightedObjectiveValue = 0;

        int coverageMapRow=0;
        int coverageMapColumn=0;

        point2 samplePoint = new point2(-1,-1);
        double miss_prob;
         double alpha;
         try
         {
             for (double horizontalSamplePoint = 0.01; horizontalSamplePoint <= boundary.largestX;
                                                 horizontalSamplePoint += objEvalIncrement)
             {
                 for (double verticalSamplePoint = 0.01; verticalSamplePoint <= boundary.largestY; verticalSamplePoint += objEvalIncrement)
                 {
                     miss_prob = 1;
                     samplePoint.x = horizontalSamplePoint;
                     samplePoint.y = verticalSamplePoint;
                   
                        for (int i5 = 0; i5 < realTimeRobotNumber; i5++)
                         {
                             if (point2.Dist(robotList.get(i5).position, samplePoint) < robotList.get(i5).sensingCutoffRange) //SYNC
                             {
                                 if (boundary.LineOfSight(robotList.get(i5).position, samplePoint))
                                 {
                                     alpha = 1; //sensing ability discount factor
                                   
                                     if(!robotList.get(i5).is_point_visible(samplePoint))
                                     {
                                    	 alpha = EventDensity.WALL_DECAY_FACTOR;
                                     }                                   
                                     
                                     if(alpha>0)
                                     {
                                     miss_prob *= (1 - 
                                     alpha * robotList.get(i5).SensingModelFunction(point2.Dist(robotList.get(i5).position, samplePoint)));
                                     }
                                 }
                             }
                         }
                    
                     coverageMap[coverageMapRow][coverageMapColumn] = miss_prob;
                     coverageMapRow++;

                     objective += (density.GetEventDensity(samplePoint) * (1 - miss_prob));
                     this.weightedObjectiveValue +=  (density.GetEventDensity(samplePoint) * WeightedDetectionProbabilityReward(1 - miss_prob));
                 }
                 coverageMapColumn++;
                 coverageMapRow = 0;
             }
         }
         catch(ArrayIndexOutOfBoundsException e)
         {
        	 e.printStackTrace();
             return;
         }

         objective = objective * objEvalIncrement * objEvalIncrement;
         weightedObjectiveValue = weightedObjectiveValue * objEvalIncrement * objEvalIncrement;

         if (coverage.evaluateObjFunc)
         {
                    coverage.realTimeObjFuncDisplay.setText(Float.toString((float)objective));
                    coverage.weightedObjFuncDisplay.setText(Float.toString((float)weightedObjectiveValue));
        }
         if(coverage.is_link_cost_mode)
         {
        	 double total_communication_cost = 0;
        	 for(int i = 0; i<this.realTimeRobotNumber; i++)
        	 {
        		 total_communication_cost += this.robotList.get(i).cost_to_base_with_data_rate;      		 
        	 }
         coverage.communication_cost_display.setText(Float.toString((float)total_communication_cost));
       //  cost_to_base_with_data_rate
         }

        return;
    }

    //TODO: redundant with Robot.detectionProbabilityReward() 
    public double WeightedDetectionProbabilityReward(double prob)
    {
        assert (prob>=0&&prob<=1);

        double ultraLowDetectionBoost=1;
        double smallDetectionBoost=1;
        try{
             ultraLowDetectionBoost = new Double(coverage.UltraLowDetectionBoostTextfield.getText());
             smallDetectionBoost = new Double(coverage.smallDetectionBoostTextfield.getText());
        }
        catch(java.lang.NumberFormatException e)
        {

        }

        if(prob<0.1)
        {
            return prob*ultraLowDetectionBoost;
        }
        else if(prob<0.5)
        {
            return (prob-0.1)*smallDetectionBoost+0.1*ultraLowDetectionBoost;
        }
        else {
            return (prob-0.5)+0.4*smallDetectionBoost+0.1*ultraLowDetectionBoost;
        }
    }
    
    public void printTree()
    {// To print the path from i to the leader 0;
    	for(int i=1; i<realTimeRobotNumber; i++)
    	{
    		int[] pathForLeader = new int[realTimeRobotNumber];
    		int num=0;
    		pathForLeader[num]=i;
    		if(robotList.get(i).predecessor!=null)
    		{
	    		int i_previous = robotList.get(i).predecessor.id;
	    		while(i_previous !=0)
	    		{
	    			num=num+1;
	    			pathForLeader[num]=i_previous;
	    			i_previous = robotList.get(i_previous).predecessor.id;
	    			
	    		}
	    		System.out.println("Robot "+ i +" path: ");
	    		for(int j=0;j<num+1;j++)
	    		{
	    			System.out.print( pathForLeader[j]+" ----> ");
	    		}
	    		System.out.println("0");
    		}
    		else
    		{
    			//System.out.println("The previous predecesor of node "+ i + robotList.get(i).predecessorCopy);
    			System.out.println("No predecessor");
    		}
    		
    	}
    }
    
     int[]  creatQueue()
    {// To find the path from 0 to i for each i
    	 int totalNumber = realTimeRobotNumber*realTimeRobotNumber;
    	 int[] Q=new int[totalNumber];
    	for(int i=1; i<realTimeRobotNumber; i++)
    	{
    		int[] pathForLeader = new int[realTimeRobotNumber];
    		int num=realTimeRobotNumber-1;
    		pathForLeader[realTimeRobotNumber-1]=i;
    		if(robotList.get(i).predecessor!=null)
    		{
	    		int i_previous = robotList.get(i).predecessor.id;
	    		while(i_previous !=0)
	    		{
	    			num=num-1;
	    			pathForLeader[num]=i_previous;
	    			i_previous = robotList.get(i_previous).predecessor.id;
	    			
	    		}
	    		for (int j=num; j<realTimeRobotNumber;j++)
	    		{
	    			Q[(i-1)*realTimeRobotNumber+j]=pathForLeader[j];
	    		}
    		}
    		else
    		{
    			//System.out.println("The previous predecesor of node "+ i + robotList.get(i).predecessorCopy);
    			System.out.println("No predecessor");
    		}
    		
    	}
    	Q=removeDupInIntArray(Q);
    	return Q;
    }
     
     int[] removeDupInIntArray(int[] ints){
    	    Set<Integer> setString = new LinkedHashSet<Integer>();
    	    int[] A= new int[realTimeRobotNumber];
    	    for(int i=0;i<ints.length;i++){
    	        setString.add(ints[i]);
    	    }
    	    //System.out.println(setString);
    	   
    	    	  int[] a = new int[setString.size()];
    	    	  int i = 0;
    	    	  for (Integer val : setString) a[i++] = val;
    	    	
	return a;
    	}

    public void updateEventCollectionPercentage()
    {
        for (int j = 0; j<this.currentDetectedEvents.size(); j++)
        {
            double currectPercentage = 0;
            for(int i = 0; i < realTimeRobotNumber; i++)
            {
                currectPercentage += robotList.get(i).monitoringModelFunction(currentDetectedEvents.get(j).eventLocation);
            }
            currentDetectedEvents.get(j).monitorPercentage = currectPercentage;
        }
    }
    
    synchronized public void reset_robots()
    {
    	for(int i = 0; i < coverage.MAX_ROBOT_NUMBER; i++)
        {	
    		if(i<robot_init_pos_list.size())
    		{
    			robotList.get(i).position.copy(robot_init_pos_list.get(i));
    			robotList.get(i).estimatedPosition.copy(robot_init_pos_list.get(i));
    		}
        	else
        	{
        		
        		robotList.get(i).position.set(0.1+0.1*i,0.1+0.1*i); 
        		robotList.get(i).estimatedPosition.set(0.1+0.1*i,0.1+0.1*i);          
        	}
    		robotList.get(i).heading = 0.0;
    		robotList.get(i).SetSensorHeading(0);
        	
    		robotList.get(i).predecessor = null;
        }
    }
   
    synchronized public void ResetObstacles()
    {
    	for(int i = 0; i < this.obstacles.size(); i++)
    	{
    		obstacles.get(i).discoveredVerticesNumber = 0;
    		if (obstacles.get(i).isAllVerticesDiscovered)
    		{
    			obstacles.get(i).isAllVerticesDiscovered = false;
    			obstacles.get(i).fillinVertices();   		
    		}
    		else
    		{
	    		for(int j = 0; j< this.obstacles.get(i).vertices.size();j++)
	    		{
	    			this.obstacles.get(i).vertices.get(j).isDiscovered = false;
	    		}
    		} 			
    	}
    	
        isAllSpaceDiscovered = false;
        mapBuildingGrid = new int[(int)(boundary.largestX/mapBuildingIncrement)][(int)(boundary.largestY/mapBuildingIncrement)];

    }

}

class UpdateObjInfoTask extends SwingWorker<Void, Void> {
	Simulation sim;
	
	public UpdateObjInfoTask(Simulation s)
	{
		sim = s;
	}
    @Override
    public Void doInBackground() {
    	  
    	sim.UpdateObjInfoActual();
    	return null;
    }

    @Override
    public void done() {
       // Toolkit.getDefaultToolkit().beep();

    }
}


