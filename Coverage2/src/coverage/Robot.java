package coverage;

import java.util.*;


public class Robot
{
    public Robot()
    {}

    //universal identifier
    int id; //start from 0

    // Physical properties
    public point2 position; //robot position
    double heading;
    double speed;
    double rightSpeed;  //not used now, but in the future, robot might have different wheel speed
    double width;
    
    //sensor FOV 
    double sensor_heading = 0;
    point2 FOV_edge_point_left = new point2();   //A point on the left edge of FOV cone. Distance to robot is sensing range
    point2 FOV_edge_point_right = new point2();
    point2 FOV_edge_impact_point_left = new point2(); //the point where left edge hits something
    point2 FOV_edge_impact_point_right = new point2();
    double FOV_degree = Math.PI*2; //was Math.PI/3*2 = 120 degree
    double FOV_edge_heading_left = sensor_heading-FOV_degree/2;
    double FOV_edge_heading_right = sensor_heading+FOV_degree/2;;
    boolean is_FOV_constrained = true;  
    double dFdh; //partial derivative w.r.t sensor heading 
    double sensor_turning_speed;
    double max_sensor_turning_speed = 0.1; //was 0.01
	
    public double robotMaxSpeed;

    //Simulation related
    double lastStatesUpdateTime;
    Simulation sim;

    //Sensing model
    public double sensingDecayFactor;

    //Control related
    public double controlPeriod;
    double lastControlTime;

    double dFdx; //TODO: not sure if these two should be here
    double dFdy;
    double dFdx2;
    double dFdy2;
    public double norminalGradientMagnitude = 1;   //if it is zero, the "speed" of a node is fixed. If it is positive, the speed is proportional to the gradient magnitude

    //control parameters
    public double surface_integral_increment = 1; //0.25 is the C++ version value; This variable will be changed by UI
    public double sensingCutoffRange = 8; //was 57
    public double smallDetectionBoost = 1; //the boost for area currently with 10%-40% joint detection probability, 1 means no boost
    public double ultraLowDetectionBoost = 1; //0%-10%, 1 means no boost

    //Environment
    Obstacle boundary;
    ArrayList<Obstacle> obstacles;
    EventDensity density;

    //Neighbors
    ArrayList<Robot> neighbors = new ArrayList<Robot>();

    //Trajectory history
    public boolean recordTrajectory = false;
    int trajectoryHistoryIndex = 0;
    int TRAJECTORY_HISTORY_SIZE = 2000; //the size is tentative, might be too large
    double trajectoryHistoryX[] = new double[TRAJECTORY_HISTORY_SIZE];
    double trajectoryHistoryY[] = new double[TRAJECTORY_HISTORY_SIZE];

    //Communication
    public double costToBase = 0;
    //public double costToPredecessor;
    public Robot predecessor = null;
    public ArrayList<Robot> descendants = new ArrayList<Robot>();
    public double his_informationValue = 0;
    
    public double potential_costToBase;
    public Robot potential_predecessor;
    public ArrayList<Robot> potential_descendants = new ArrayList<Robot>();
    
    protected double ri; //amount of information collected by me
    protected double zi; //amount of information going through me
    public double communicationCostWeight = 0;  //0 means we do not worry about communication cost
    protected final int bitPerDetection = 1; //the alpha3 in Li's paper
    protected double cost_to_base_with_data_rate;  //costToBase times the data rate, which is related to the area a node can cover

    //Position estimation, asynchronous stuff
    //sensor heading should be estimated too
    public point2 estimatedPosition= new point2(0,0);
    public point2 localEstimatedPosition= new point2(0,0);
    public point2 lastPosition= new point2(0,0);
    public double lastHeading;
    public double lastSpeed;
    public double lastBroadcastTime;
    public double estimationErrorThreshold = 0; //if the position estimation error exceeds this threshold, actual position and heading should be broadcasted. Default is 0
    public boolean showRed = true;
    public  double K4 = 0;
    public boolean isFixedError = false;

    //event info collection stuff
    public double dataCollectionDecay = 0.09;
    public double dataCollectionWeight = 1000;

    //determine when to stop broadcasting due to small gradient
    protected boolean isMyGradientMagSmall = false;
    protected double SMALL_GRADIENT_THRESHOLD = 0;//0.000000000001;   //if it is zero, the broadcast will not stop due to small gradient

    //delay stuff
    public int comm_delay = 0; //TODO: why we have todo here?
    protected Queue<CommunicationMessage> message_queue = new LinkedList<CommunicationMessage>();
    Iterator<CommunicationMessage> message_iterator;
    CommunicationMessage tempMessage;
    
    //connectivity stuff
	public boolean is_upstream_connectivity_in_danger = false;
	public boolean is_stretching_upstream = false;
	
	//seeker stuff, move to the location of another robot
	boolean is_seeker_mode = false;  
	Robot seeker_target = null;
	double seeking_hit_range = 0;  //if set as 0, robot will never switch out of seeker mode by itself
	
	//map building
	public boolean inActiveMapBuildingMode = false;

	public boolean UpdatedState = false;
	
    
    public Robot(int _id, point2 p, double h, double leftS, double rightS, double w, double robotMaxS, double lastStatesUpdateT,
                 double controlP, double lastControlT, Obstacle bndr, ArrayList<Obstacle> gswl, EventDensity dnst,
            Simulation s, double sensing_decay)
    {
        id = _id;
        position = p;
        heading = h;
        speed = leftS;
        rightSpeed = rightS;
        width = w;
        robotMaxSpeed = robotMaxS;
        lastStatesUpdateTime = lastStatesUpdateT;
        controlPeriod = controlP;
        lastControlTime = lastControlT;
        boundary = bndr;
        density = dnst;
        obstacles = gswl;
        dFdx = 0;
        dFdy = 0;
        sim = s;
        sensingDecayFactor = sensing_decay;
        
        this.sensor_heading = id; //separate starting sensor heading;
        set_FOV_edge_points();
        
    }

    void UpdateStates(double currentTime) //update position and heading based on the current speed and (currentTime-lastStatesUpdateTime), should be called at every simulation step
    {
        double updateInterval = currentTime - lastStatesUpdateTime;

//not perfect vehicle handling
        /*    double dw = (rightSpeed-leftSpeed)/width;		//heading increment
            double dx = ((rightSpeed+leftSpeed)/2)*cos(heading+dw/2);

            double dy = ((rightSpeed+leftSpeed)/2)*sin(heading+dw/2);

            position.x += dx*updateInterval;
            position.y += dy*updateInterval;
            heading += dw*updateInterval;*/


//perfect vehicle handling
        
        //seeker mode
        if(this.is_seeker_mode == true)
        {
        	if(point2.Dist(this.position,this.seeker_target.position)<this.seeking_hit_range)
        	{
        		is_seeker_mode = false;
        		return;
        	}
        	heading = Math.atan2(this.seeker_target.position.y-position.y,this.seeker_target.position.x-position.x);
        	point2 tempPosition = new point2(position.x + speed * Math.cos(heading) * updateInterval,
                    position.y + speed * Math.sin(heading) * updateInterval);
        	
        	obstacle_sliding(updateInterval, tempPosition);
        	RecordTrajectory();
        	location_estimation_handling(currentTime);
        	        
        	lastStatesUpdateTime = currentTime;
        	return; //do not go to normal mode
        }
        
        //normal mode
        
        //the next robot position
        point2 tempPosition = new point2(position.x + speed * Math.cos(heading) * updateInterval,
                                         position.y + speed * Math.sin(heading) * updateInterval);
//        System.out.println("Robot ID is "+ id+" tempPosition is "+ tempPosition.x + " "+ tempPosition.y);
        //make sure nodes have connectivity.
        // IMS constraint
        if(sim.coverage.has_connectivity_constraint && sim.BreadthFirst)
        {
        	System.out.println("Run me???"+sim.BreadthFirst);
        	double distance = point2.Dist(tempPosition, sim.robotList.get(this.predecessor.id).position);
        	sim.path.buildRoutingTree();
        	// Test whether in the middle of two layers.
        	if(  sim.communication_range< distance)
        	{
        		tempPosition = point2.plus(sim.robotList.get(this.predecessor.id).position, 
        				point2.divide( point2.minus(tempPosition, sim.robotList.get(this.predecessor.id).position),
        						distance/(0.9999*sim.communication_range)));
        		
        	}
        	
        
        }
    
        if(sim.coverage.has_connectivity_constraint && sim.RandomOrder) 
        {
//        	System.out.println("robot "+this.id + "'s predecessor is "+ this.predecessor.id);
        	
        	if(this.predecessor == null)
        	{
        		// *If there is no predecessor, nodes will go to the base.
//        		sim.pathGenerator.buildRoutingTree();
//        		if(this.predecessor == null)
        		{
        			heading = Math.atan2(sim.robotList.get(0).position.y-position.y,sim.robotList.get(0).position.x-position.x);
	        		tempPosition.x = (position.x + speed * Math.cos(heading) * updateInterval);
	        		tempPosition.y = (position.y + speed * Math.sin(heading) * updateInterval);
        		}
        	}
        	else //already have a predecessor
        	{
        		is_upstream_connectivity_in_danger = false;
        		
        		double temp_predecessor_distance = point2.Dist(tempPosition, this.predecessor.position);
        		
		        //if(temp_predecessor_distance>sim.communication_range)
		        {	
		        	point2 old_position = new point2(this.position);
		        	//Robot old_predecesssor = this.predecessor;
		        	this.position.copy(tempPosition); //TODO: why modify this.position, can't we just use potential_position?
		        	sim.pathGenerator.build_potential_routing_tree();
		        	
		        	 //just for debuging
		          //  System.out.println("B: robot "+id + "'s predecessor is "+ this.predecessor.id);
		           		            
		        	this.position.copy(old_position);
		        	
		        	if(this.potential_predecessor != null) //can find a alternative routing path to base station after the next step
		        	{
		        		this.predecessor = this.potential_predecessor;
		        	}
		        	else   // can not find a communication neighbor after the next step, try to keep contact with current predecessor      	
		        	{
		        		is_upstream_connectivity_in_danger = true;
		        	
		        		if(temp_predecessor_distance<=sim.communication_range)
		        		{//link broken caused by obstacle
		        			if(point2.dot(point2.minus(this.position, this.predecessor.position), 
		        					new point2(Math.cos(heading),Math.sin(heading))) >= 0)
		        			{
		        				tempPosition=point2.plus(this.position,point2.product(point2.normalize(point2.minus(
		        								this.position, this.predecessor.position)),0.05));		        		
		        			}
		        		}
		        		else
			        	{			        		
		        			//slide along the circle. Is the sliding direction always a cost-reducing direction?
			        		tempPosition = point2.plus(this.predecessor.position, 
			        				point2.divide( point2.minus(tempPosition, this.predecessor.position),
			        						temp_predecessor_distance/(0.9999*sim.communication_range)));
		        			
			        		//test if  the sliding is reward increasing
//			        		double original_obj = this.sim.EvaluateObjectiveFunc();
//			        		point2 temp_point2 = this.position;
//			        		this.position = tempPosition;
//			        		double new_obj = this.sim.EvaluateObjectiveFunc();
//			        		this.position = temp_point2;
//			        		if(new_obj<original_obj)
//			        		{
//			        			System.out.println("original is "+ original_obj+ "and new is "+new_obj);
//			        			System.out.println("temp: "+tempPosition);
//			        			tempPosition.x = position.x;
//				        		tempPosition.y = position.y;
//			        		}
			        		//test end	        		
		        		
				        	//just for debug
		//		        	if(point2.Dist(tempPosition, this.predecessor.position)>=sim.communication_range)
		//		        	{
		//		        		System.out.println(point2.Dist(tempPosition, this.predecessor.position)+"failure");
		//		        	}
				        	//System.out.println(predecessor.position.x +"  "+predecessor.position.y);	        	
				        }
		        		// * Why this if happens? tempPosition is projected on the circle.
		        		if(point2.Dist(tempPosition, this.predecessor.position)>=sim.communication_range)
        				{
		        			tempPosition.copy(position);
        				}
		        		else
		        		{
			        		for (int i2 = 0; i2 < obstacles.size(); i2++)
			                {
			                       if (!obstacles.get(i2).LineOfSight(tempPosition,this.predecessor.position)&&
			                    		 (obstacles.get(i2).LineOfSight(tempPosition,this.position)))
			                       {
			                    	   tempPosition.copy(position);
			                           break;
			                       }
			                }
		        		}		        		
		        	}
		        }
        	
        	
		        boolean is_descendant_broken = false;
		        //TODO should test new descendants, but since sliding change tempPosition, can not use the current potential_descendants  
		        //TODO if only one descendant is stretched (and predecessor is not stretched), try slide on the circle of that descendant
		        for(int i =0;i<this.descendants.size();i++)
		        {
		        	this.descendants.get(i).is_stretching_upstream=false;		       
		        	
		        	if(point2.Dist(tempPosition, this.descendants.get(i).position)>=sim.communication_range)
		        	{
		        		this.descendants.get(i).is_stretching_upstream=true;
		        		is_descendant_broken = true;	        		
		        	}
		        	else
		        	{
			        	for (int i2 = 0; i2 < obstacles.size(); i2++)
		                {
		                       if (!obstacles.get(i2).LineOfSight(tempPosition,this.descendants.get(i).position))
		                       {
		                    	   this.descendants.get(i).is_stretching_upstream=true;
		                    	   is_descendant_broken = true;	                    	  
		                           break;
		                       }
		                }
		        	}
		        }
		        
		      	if(is_descendant_broken)
	        	{
		        	if(!is_upstream_connectivity_in_danger && descendants.size()==1)
		        	{
		        		tempPosition = point2.plus(descendants.get(0).position, 
		        				point2.divide( point2.minus(tempPosition, descendants.get(0).position),
		        						point2.Dist(tempPosition, descendants.get(0).position)/(0.9999*sim.communication_range)));
		        		
		        		if(point2.Dist(tempPosition, this.predecessor.position)>=sim.communication_range)
        				{
		        			tempPosition.copy(position);
        				}
		        		else
		        		{
			        		for (int i2 = 0; i2 < obstacles.size(); i2++)
			                {
			                       if (!obstacles.get(i2).LineOfSight(tempPosition,this.predecessor.position)&&
			                    		 (obstacles.get(i2).LineOfSight(tempPosition,this.position)))
			                       {
			                    	   tempPosition.copy(position);
			                           break;
			                       }
			                }
		        		}		      
		        	}
		        	else
		        	{
		        		tempPosition.copy(position);
		        	}
		        //TODO: some room for optimization
	        	}
        	}
        }
        
        if(tempPosition.x > sim.robotList.get(0).position.x  )
        {
        	tempPosition.x = sim.robotList.get(0).position.x;
        }
      obstacle_sliding(updateInterval, tempPosition);
      RecordTrajectory();
      
      this.modify_sensor_heading(updateInterval*this.sensor_turning_speed);

//end vehicle handling

        location_estimation_handling(currentTime);
        
        if (currentTime - lastControlTime >= controlPeriod)
        {
            sim.requestCount += (sim.realTimeRobotNumber - 1); //used for counting communication frequency
            SpeedControlAlgo(currentTime);
            lastControlTime = currentTime + Math.random() * 0.05 - 0.025;   //noisy control period
            //lastControlTime = currentTime;       //precise control period
        }
        
        lastStatesUpdateTime = currentTime;
    }
private void RecordTrajectory()
{
    if (this.recordTrajectory)
    {
        if (Math.abs(position.x - this.trajectoryHistoryX[trajectoryHistoryIndex]) +
            Math.abs(position.y - this.trajectoryHistoryY[trajectoryHistoryIndex]) > 2) //2 is recording threshold, adjustable
        {
            trajectoryHistoryIndex++;
            if(trajectoryHistoryIndex==TRAJECTORY_HISTORY_SIZE)
            {
                trajectoryHistoryIndex = 0; //start all over.
            }
            this.trajectoryHistoryX[trajectoryHistoryIndex] = position.x;
            this.trajectoryHistoryY[trajectoryHistoryIndex] = position.y;
        }
    }
}
    
	private void location_estimation_handling(double currentTime) {
		//handle position estimation
        //using estimation might reduce the number of communication needed. But it seems that convergence will not be guaranteed. So it is disabled for now
        //uncomment the following two lines to use linear estimation, TODO: adding message_queue might make the following incorrect
     //   this.position.x = this.lastPosition.x + lastSpeed * Math.cos(lastHeading) * (currentTime - this.lastBroadcastTime);
      //  this.position.y = this.lastPosition.y + lastSpeed * Math.sin(lastHeading) * (currentTime - this.lastBroadcastTime);

		//TODO: need to broadcast sensor_heading 
        //re-broadcast if necessary. 1: my position estimation exceeds a threshold 2: my gradient magnitude is still big enough
        if ((point2.Dist(localEstimatedPosition, position) > estimationErrorThreshold) && !isMyGradientMagSmall)
        {
        	this.message_queue.offer(new CommunicationMessage(position, this.heading, this.comm_delay));
        	
        	localEstimatedPosition.copy(position);
        	lastPosition.copy(position);      	
            this.lastSpeed = this.speed;
            this.lastHeading = this.heading;
            this.lastBroadcastTime = currentTime;
            showRed = !showRed;
            sim.broadcastCount++; //for comparison
            //System.out.println("robot "+this.id+" broadcasted at "+currentTime);
        }
        
        //check if a new message can be reached by other nodes, out of the waiting queue
        tempMessage = message_queue.peek();
        if(tempMessage!=null && tempMessage.remaining_delay<=0)
        {
        	this.estimatedPosition.x = tempMessage.x;
        	this.estimatedPosition.y = tempMessage.y;
        	message_queue.poll();
        }
        message_iterator = message_queue.iterator();
        while(message_iterator.hasNext())
        {
        	message_iterator.next().remaining_delay--;
        }
	}

	private void obstacle_sliding(double updateInterval, point2 tempPosition) {
		//detect if a robot runs into boundary or an obstacle P3 TODO: BSP tree to improve efficiency
		    boolean runIntoObstacle = false;
		    point2 direction = new point2(0, 0); //direction will be filled by LineOfSight
		   
		    //test if robot runs into boundary		    
		    if(tempPosition.x>boundary.largestX)
		    {
		    	tempPosition.x = boundary.largestX-0.001;
		    }
		    else if(tempPosition.x<boundary.smallestX)
		    {
		    	tempPosition.x = boundary.smallestX+0.001;
		    }
		    
		    if(tempPosition.y>boundary.largestY)
		    {
		    	tempPosition.y = boundary.largestY-0.001;
		    }
		    else if(tempPosition.y<boundary.smallestY)
		    {
		    	tempPosition.y = boundary.smallestY+0.001;
		    }	 

		    //if not into boundary, test if it runs into any obstacles, if it does, it will stop for one turn but the heading will be set properly   
		    for (int i1 = 0; (i1 < obstacles.size()) && (!runIntoObstacle); i1++)
		    {
		    	//TODO: there is a bug: if a node cross two boundaries of an obstacle, the direction of the
		    	//first one tested will be returned, not the closest's, a workaround is to describe obstacles
		    	//starting from the lower right corner.
		    	//TODO: another bug is that if a node cross boundaries of two obstacles
		        if (!(obstacles.get(i1).LineOfSight(position, tempPosition, direction)))
		        {
		            if (point2.dot(point2.minus(tempPosition, position), direction) >= 0)
		            {
		                heading = Math.atan2(direction.y, direction.x);                   
		            }
		            else
		            {
		                heading = Math.atan2( -direction.y, -direction.x);
		            }
		            runIntoObstacle = true;
		            tempPosition.copy(this.position); 
		            tempPosition.shift(speed * Math.cos(heading) * updateInterval,
		                    speed * Math.sin(heading) * updateInterval); 
		            
		            if(isPointInLOS(tempPosition))
		            {
		            	 this.position.copy(tempPosition);  
		            }
		            
		            //position.shift(speed * Math.cos(heading) * updateInterval, speed * Math.sin(heading) * updateInterval);	              
		            return;
		        }
		    }
		    
		    if (runIntoObstacle == false)
		    {
		        position.copy(tempPosition); 
		    }
	}

    void PrintRobotInfo()
    {
        System.out.print("robot " + id + " @ " + position.x + "," + position.y + "," + heading + "," +sensor_heading + "," + speed);
    }

    public boolean is_point_visible(point2 sample_point) //Notice that a point coincide with robot position is NOT visible
    {
    	double distance = point2.Dist(sample_point, position);
        
        if (distance > this.sensingCutoffRange || distance <= 0) //to make sure dist>0 is necessary. SamplePoint might coincide with robot position. dist=0 will cause divide by zero
        {	
        	return false;
        }
    	
        //boundary block all sensing. This test can be disabled if nodes can not stay out of boundary and boundary is always a rectangle.  
        if (!boundary.LineOfSight(position, sample_point)) return false;

        if(!isPointInFOV(sample_point))
        {
        	return false;
        }
        
        if(!isPointInLOS(sample_point))
        {
        	return false;
        }
              
    	return true;
    }
    
    public boolean isPointInLOS(point2 samplePoint) //in line of sight, not considering FOV, only obstacles
    {
    	 for (int i1 = 0; i1 < obstacles.size(); i1++)
         {
             if (!obstacles.get(i1).LineOfSight(position, samplePoint))
             {
                 return false;
             }
         }
    	 return true;
    }
   
    // return blocked obstacles number
    public BlockedObstacle blockedObs(point2 samplePoint)
    {
    	BlockedObstacle blockObs = new BlockedObstacle(-1,-1);
    	 for (int i1 = 0; i1 < obstacles.size(); i1++)
         {
    		 int verticeNum =obstacles.get(i1).noLineOfSightVerticeNum(position, samplePoint); 
             if (verticeNum>-1)
             {
                 blockObs.obstacleNum=i1;
                 blockObs.verticeNum = verticeNum;
                 return blockObs;
             }
         }
    	 return blockObs;
    }
    
    public boolean isPointInFOV(point2 samplePoint)
    {
        //If change is made here, make sure both FOV < 180 and >180 cases work correctly
        if(this.is_FOV_constrained && this.FOV_degree<2*Math.PI)
        {
        	if (this.FOV_degree<Math.PI)
        	{
        		if (point2.cross(point2.minus(FOV_edge_point_left, position), point2.minus(samplePoint,position))<0 
        				|| point2.cross(point2.minus(FOV_edge_point_right, position), point2.minus(samplePoint,position))>0)
        		{
        			return false;
        		}
        	}
        	else
        	{
        		if (point2.cross(point2.minus(FOV_edge_point_left, position), point2.minus(samplePoint,position))<0 
        				&& point2.cross(point2.minus(FOV_edge_point_right, position), point2.minus(samplePoint,position))>0)
        		{
        			return false;
        		}
        	}
        }
    
        return true;
    }
    
    
    
    synchronized void SpeedControlAlgo(double currentTime) //using control algorithm to set the robot's speed. Called every control period, not every simulation step
    {
    	if(this.is_FOV_constrained && this.FOV_degree<2*Math.PI)
    	{
    		FastSearchSensorHeading();
    	}
        //performance tuning:
        //MyTimer myTimer = new MyTimer();
        //long beginTime = myTimer.currentTimeMillis();
        dFdx = 0;
        dFdy = 0;
        ri = 0;

        double temp;

        double delta = surface_integral_increment; //0.25 is our original value
        point2 samplePoint = new point2( -1, -1);
        double neighborEffects;
        double dist;
        double alpha;
        double Phat, Phat0;
        
        double sample_upper_bound_x = Math.min(position.x+sensingCutoffRange, boundary.largestX);
        double sample_upper_bound_y = Math.min(position.y+sensingCutoffRange, boundary.largestY);
        
        for (double horizontalSamplePoint =  Math.max(position.x-sensingCutoffRange, 0.01);horizontalSamplePoint<=sample_upper_bound_x; horizontalSamplePoint+=delta)
        {
            for (double verticalSamplePoint = Math.max(position.y-sensingCutoffRange, 0.01); verticalSamplePoint<=sample_upper_bound_y; verticalSamplePoint+=delta)
            {
                samplePoint.x = horizontalSamplePoint;
                samplePoint.y = verticalSamplePoint;
                dist = point2.Dist(samplePoint, position);
                      
                if(is_point_visible(samplePoint))
                {
                	alpha = 1;
                }
                else
                {
                	alpha = EventDensity.WALL_DECAY_FACTOR;
                }
                
                if (alpha > 0) //performance concern, often EventDensity.WALL_DECAY_FACTOR is 0;
                {
                //	if(dist <= 0)
                //	{
                //		System.out.println("dist "+dist);
                //	}
                	
                    //take care of ri along the way
                    ri += alpha * density.GetEventDensity(samplePoint, currentTime) * SensingModelFunction(dist);

                    neighborEffects = NeighborEffects(samplePoint);
                    temp = density.GetEventDensity(samplePoint, currentTime) * alpha * SensingModelDerivative(dist) *
                           neighborEffects / dist;

                    //boost low detection area
                    Phat = 1 - neighborEffects * (1 - SensingModelFunction(dist));
                    if (Phat < 0.1)
                    {
                        temp = temp * ultraLowDetectionBoost;
                    }
                    else if (Phat < 0.5)
                    {
                        temp = temp * smallDetectionBoost;
                    }
                    //temp = temp*(1-Phat)*(1-Phat)*(1-Phat);//continuous low detection boost

                    //if (((horizontalSamplePoint - position.x) > 0.01) || ((horizontalSamplePoint - position.x) < -0.01))
                    {
                        dFdx += temp * (horizontalSamplePoint - position.x) * delta * delta;
                    }
                    
                    // if (((verticalSamplePoint - position.y) > 0.01) || ((verticalSamplePoint - position.y) < -0.01))
                    {
                        dFdy += temp * (verticalSamplePoint - position.y) * delta * delta;
                    }
                }
            }
        }

        //long midTime = myTimer.currentTimeMillis();  //for performance tuning

        dFdx2 = 0;
        dFdy2 = 0;

        ArrayList<point2> activeReflexVertices = new ArrayList<point2>();

        //Assuming the boundary is convex

        for (int i2 = 0; i2 < obstacles.size(); i2++)
        {
            obstacles.get(i2).GetActiveReflexVertices(position, activeReflexVertices); //must clear activeReflexVertices before calling this again

            for (int i3 = 0; i3 < activeReflexVertices.size(); i3++)
            {
                double D = point2.Dist(activeReflexVertices.get(i3), position); 
                if (D < this.sensingCutoffRange && isPointInFOV(activeReflexVertices.get(i3))) 
                {
                    boolean ARVBlocked = false; //test if an ARV is blocked by other FG
                    if (!(boundary.LineOfSight(position, activeReflexVertices.get(i3))))
                    {
                        ARVBlocked = true;
                    }
                    else
                    {
                        for (int i4 = 0; i4 < obstacles.size(); i4++)
                        {
                            if (i2 != i4)
                            {
                                if (!(obstacles.get(i4).LineOfSight(position, activeReflexVertices.get(i3))))
                                {
                                    ARVBlocked = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!ARVBlocked)
                    {
                        //SYNC
                        point2 impactPoint = new point2( -1, -1); //it should be filled with useful values before used by algorithm
                        boundary.GetImpactPoint(activeReflexVertices.get(i3), point2.ExtendToInfinite(position, activeReflexVertices.get(i3)),
                                                impactPoint, true);

                        for (int i5 = 0; i5 < obstacles.size(); i5++)
                        {
                            if (i5 != i2)
                            {
                                obstacles.get(i5).GetImpactPoint(activeReflexVertices.get(i3),
                                        point2.ExtendToInfinite(position, activeReflexVertices.get(i3)), impactPoint, false);
                            }
                        }
                        double d = point2.Dist(impactPoint, activeReflexVertices.get(i3));
                        //System.out.println("anchor:"+activeReflexVertices.get(i3).x+","+activeReflexVertices.get(i3).y);

                        double SinTheta = Math.abs(position.y - activeReflexVertices.get(i3).y) / D;
                        double CosTheta = Math.abs(position.x - activeReflexVertices.get(i3).x) / D;

                        point2 pointingOutsideNorm;
                        double sign = 1;
                        if (point2.dot(point2.minus(position, activeReflexVertices.get(i3)).Norm1(),
                                       (point2.minus(obstacles.get(i2).interiorPoint, activeReflexVertices.get(i3)))) > 0)
                        {
                            pointingOutsideNorm = point2.minus(position, activeReflexVertices.get(i3)).Norm2();
                        }
                        else
                        {
                            pointingOutsideNorm = point2.minus(position, activeReflexVertices.get(i3)).Norm1();
                        }
                        if (pointingOutsideNorm.x < 0)
                        {
                            sign = -1;
                        }

                        //Line numerical integration
                        double alpha1 = EventDensity.WALL_DECAY_FACTOR;
                        double integralSum = 0;
                        double delta1 = 0.2; //was 0.05 
                        double integrationUpperBound = d; 
                        if ((D + d) > this.sensingCutoffRange)
                        {
                            integrationUpperBound = this.sensingCutoffRange - D;
                        }

                        point2 integralPoint = new point2();
                        for (double r = delta1 / 2; r <= integrationUpperBound; r += delta1) 
                        {
                            //point2 integralPoint = (impactPoint-activeReflexVertices.get(i3))*r/d+activeReflexVertices.get(i3);
                            // integralPoint = point2.plus(point2.divide(point2.product(point2.minus(impactPoint,
                            //        activeReflexVertices.get(i3)), r), d), activeReflexVertices.get(i3));
                            integralPoint.x = (impactPoint.x - activeReflexVertices.get(i3).x) * r / d + activeReflexVertices.get(i3).x;
                            integralPoint.y = (impactPoint.y - activeReflexVertices.get(i3).y) * r / d + activeReflexVertices.get(i3).y;

                            neighborEffects = NeighborEffects(integralPoint);

                            //boost low detection area
                            //visible
                            Phat0 = 1 - neighborEffects * (1 - SensingModelFunction(D + r));
                            //invisible
                            Phat = 1 - neighborEffects * (1 - alpha1 * SensingModelFunction(D + r)); //detection probability by neighbors
                            integralSum += delta1 * r * density.GetEventDensity(integralPoint, currentTime) *
                                    (this.detectionProbabilityReward(Phat0) - this.detectionProbabilityReward(Phat));
                        }

                      
                        //two debug variables
                        //double temp1 = sign*SinTheta/D*(-d/sensingDecayFactor*SensingModelFunction(d+D)-1/sensingDecayFactor/sensingDecayFactor*(SensingModelFunction(d+D)-SensingModelFunction(D)));
                        //double temp2 = sign*SinTheta/D*integralSum;

                        dFdx2 += sign * SinTheta / D * integralSum;

                        if (pointingOutsideNorm.y < 0)
                        {
                            sign = -1;
                        }
                        else
                        {
                            sign = 1;
                        }
                        //dFdy2+=sign*CosTheta/D*(-d/sensingDecayFactor*SensingModelFunction(d+D)-1/sensingDecayFactor/sensingDecayFactor*(SensingModelFunction(d+D)-SensingModelFunction(D)));

                        dFdy2 += sign * CosTheta / D * integralSum;
                    }
                }
            }
            activeReflexVertices.clear(); //this container only holds ARV from one obstacle, so when we move to another obstacle, we need to empty it first.
        }

        // performance tuning code
        //  long endTime  = myTimer.currentTimeMillis();
        //  System.out.println("First portion:"+(midTime-beginTime));
        //  System.out.println("Second portion:"+(endTime-midTime));

        //Communication cost
        double dFdx4 = 0;
        double dFdy4 = 0;
        
        if(sim.coverage.is_link_cost_mode && this.predecessor != null)
        {
	        ri = ri * delta * delta * bitPerDetection;
	        cost_to_base_with_data_rate = ri*this.costToBase;  //calculate this along the way, it's for evaluating the obj function
	        zi = ri;
	        for (int i = 0; i < this.descendants.size(); i++)
	        {
	            zi += descendants.get(i).zi;
	        }
	        point2 obstacleBlockingGradient = new point2(0, 0);
	        point2 additionalBlockingGradient = new point2(0, 0);
	        double obstacleBlockingDistance = 0;
	        for (int i5 = 0; i5 < obstacles.size(); i5++)
	        {
	            obstacleBlockingDistance +=
	                    obstacles.get(i5).IntersectionSegmentGradient(position, predecessor.position, additionalBlockingGradient);
	
	            obstacleBlockingGradient = point2.plus(obstacleBlockingGradient, additionalBlockingGradient);
	        }
	         dFdx4 = sim.pathGenerator.link_cost_gradient_x(zi,position,predecessor,obstacleBlockingDistance,obstacleBlockingGradient);
	         dFdy4 = sim.pathGenerator.link_cost_gradient_y(zi,position,predecessor,obstacleBlockingDistance,obstacleBlockingGradient);
	
	        for (int i = 0; i < this.descendants.size(); i++)
	        {
	            obstacleBlockingGradient = new point2(0, 0);
	            obstacleBlockingDistance = 0;
	            for (int i5 = 0; i5 < obstacles.size(); i5++)
	            {
	                obstacleBlockingDistance +=
	                        obstacles.get(i5).IntersectionSegmentGradient(position, descendants.get(i).position, additionalBlockingGradient);
	                obstacleBlockingGradient = point2.plus(obstacleBlockingGradient, additionalBlockingGradient);
	            }
	
	            dFdx4 += sim.pathGenerator.link_cost_gradient_x(descendants.get(i).zi,position,descendants.get(i),obstacleBlockingDistance,obstacleBlockingGradient);
	                    
	            dFdy4 += sim.pathGenerator.link_cost_gradient_y(descendants.get(i).zi,position,descendants.get(i),obstacleBlockingDistance,obstacleBlockingGradient);
	                   // (2 * ShortestPath.Eamp * descendants.get(i).zi * (position.y - descendants.get(i).position.y) +
	                    // 4 * ShortestPath.Eamp2 * descendants.get(i).zi * Math.pow(obstacleBlockingDistance, 3) * obstacleBlockingGradient.y);
	        }
	
	        //  System.out.println("ri  "+Double.toString(ri)+"  zi  "+Double.toString(zi));
	        //   System.out.println(dFdx4);
	        //    System.out.println(dFdy4);
        }
        
        //collecting information from detected events
        //TODO: assuming no obstacles here, add visibility test later

        double dFdx3 = 0;
        double dFdy3 = 0;
        for (int i = 0; i < sim.currentDetectedEvents.size(); i++)
        {
            point2 eventLocation = sim.currentDetectedEvents.get(i).eventLocation;
            dFdx3 += monitoringModelDerivative(sim.currentDetectedEvents.get(i))*(eventLocation.x-position.x)/point2.Dist(eventLocation,this.position);
            dFdy3 += monitoringModelDerivative(sim.currentDetectedEvents.get(i))*(eventLocation.y-position.y)/point2.Dist(eventLocation,this.position);
        }
     //   System.out.println("dFdx3 = "+dFdx3 + " dFdy3 = " + dFdy3);

        //collecting information from detected events ends

        //FOV boundary
        boundary.GetImpactPoint(position, point2.ExtendToInfinite(position, FOV_edge_point_left),
        		FOV_edge_impact_point_left, true);
        
        boundary.GetImpactPoint(position, point2.ExtendToInfinite(position, FOV_edge_point_right),
        		FOV_edge_impact_point_right, true);
        
        for (int i5 = 0; i5 < obstacles.size(); i5++)
        {
        	obstacles.get(i5).GetImpactPoint(position,
        			point2.ExtendToInfinite(position, FOV_edge_point_left), FOV_edge_impact_point_left, false);
        	
        	obstacles.get(i5).GetImpactPoint(position,
        			point2.ExtendToInfinite(position, FOV_edge_point_right), FOV_edge_impact_point_right, false);
        }
        
        double FOV_edge_length_left = Math.min(point2.Dist(position, FOV_edge_impact_point_left),sensingCutoffRange);
        double FOV_edge_length_right = Math.min(point2.Dist(position, FOV_edge_impact_point_right),sensingCutoffRange);;
   
        // Line numerical integration
		double dFdx5 = 0;
		double dFdy5 = 0;
		double integral_sum_h = 0;
		double integral_sum_x = 0;
		double integral_sum_y = 0;
		double delta2 = 0.05; // was 0.2
		// double integrationUpperBound = d;
		point2 integral_point;

		for (double r = delta2 / 16; r <= FOV_edge_length_left; r += delta2) {
			integral_point = point2.plus(position, point2.divide(point2.product(
							point2.minus(FOV_edge_impact_point_left, position),
							r), FOV_edge_length_left));
			neighborEffects = NeighborEffects(integral_point);
			Phat0 = 1 - neighborEffects * (1 - SensingModelFunction(r));
			Phat = 1 - neighborEffects;
			//Phat = 1 - neighborEffects * (1 - SensingModelFunction(r));
			//Phat = neighborEffects*SensingModelFunction(r);
			temp = delta2 * density.GetEventDensity(integral_point, currentTime)
			* (this.detectionProbabilityReward(Phat0)-this.detectionProbabilityReward(Phat));
			
			integral_sum_h += temp * r;	
			integral_sum_x += temp*Math.sin(this.FOV_edge_heading_left);
			integral_sum_y += temp*Math.cos(this.FOV_edge_heading_left);
		}

		dFdh = integral_sum_h;
		dFdx5 = -integral_sum_x;
		dFdy5 = integral_sum_y;
		
//		System.out.println(this.id+": integral_sum_h_left="+integral_sum_h);
//		System.out.println(this.id+": integral_sum_x_left="+integral_sum_x);
//		System.out.println(this.id+": integral_sum_y_left="+integral_sum_y);
		
		integral_sum_h = 0;
		integral_sum_x = 0;
		integral_sum_y = 0;

		for (double r = delta2 / 16; r <= FOV_edge_length_right; r += delta2) {
			integral_point = point2.plus(position, point2.divide(point2
					.product(point2
							.minus(FOV_edge_impact_point_right, position), r),
					FOV_edge_length_right));
			neighborEffects = NeighborEffects(integral_point);
			
			Phat0 = 1 - neighborEffects * (1 - SensingModelFunction(r));
			Phat = 1 - neighborEffects;
			//Phat = neighborEffects*SensingModelFunction(r);
			//Phat = 1 - neighborEffects * (1 - SensingModelFunction(r));
			temp = delta2 * density.GetEventDensity(integral_point, currentTime)
			* (this.detectionProbabilityReward(Phat0)-this.detectionProbabilityReward(Phat));
			
			integral_sum_h += temp * r;	
			integral_sum_x += temp*Math.sin(this.FOV_edge_heading_right);
			integral_sum_y += temp*Math.cos(this.FOV_edge_heading_right);
		}
//		System.out.println(this.id+": integral_sum_h_right="+integral_sum_h);
//		System.out.println(this.id+": integral_sum_x_right="+integral_sum_x);
//		System.out.println(this.id+": integral_sum_y_right="+integral_sum_y);
		
		dFdh -= integral_sum_h;
		dFdx5 += integral_sum_x;
		dFdy5 -= integral_sum_y;
		
		if(dFdh>0)
		{
			this.sensor_turning_speed = -Math.min(this.max_sensor_turning_speed,dFdh);
		}
		else 
		{
			this.sensor_turning_speed = Math.min(this.max_sensor_turning_speed,-dFdh);
		}

		//shift_sensor_heading(dFdh);

		// dFdx2 += sign * SinTheta / D * integralSum;

		// dFdy2 += sign * CosTheta / D * integralSum;

		dFdx = -dFdx + dFdx2 - this.communicationCostWeight * (dFdx4)
				- this.dataCollectionWeight * dFdx3 - dFdx5;
		dFdy = -dFdy + dFdy2 - this.communicationCostWeight * (dFdy4)
				- this.dataCollectionWeight * dFdy3 - dFdy5;

		// System.out.println("Node"+this.id+"\tdFdx1:"+(-dFdx)+"\t\tdFdy1:"+(-dFdy)+"\t\tdFdx2:"+(dFdx2)+"\t\tdFdy2:"+(dFdy2));

        	
        //*****************************************
        //Let robot go to fixed target
        //float headingError = fmod(atan2(70-position.y,30-position.x)-heading,(float)(2*M_PI));

        //non-perfect vehicle handling
        /*
         float headingError = fmod(atan2(-dFdy,-dFdx)-heading,(float)(2*M_PI));

                        if (headingError>M_PI)
                        {
                                headingError = headingError - 2*M_PI;
                        }

                        if (headingError<-M_PI)
                        {
                                headingError = headingError + 2*M_PI;
                        }

                        if (headingError < 0)
                        {
                                leftSpeed = robotMaxSpeed;
         rightSpeed = robotMaxSpeed + robotMaxSpeed*headingError/M_PI;
                        }
                        else
                        {
         leftSpeed = robotMaxSpeed - robotMaxSpeed*headingError/M_PI;
                                rightSpeed = robotMaxSpeed;
                        }*/

        // perfect vehicle handling
        double gradientMagnitude = Math.sqrt(dFdx * dFdx + dFdy * dFdy);

        //if my gradient is small, in the future, I do not need to broadcast my position since it doesn't change much.
        if(gradientMagnitude<SMALL_GRADIENT_THRESHOLD)
        {
            isMyGradientMagSmall = true;
        }
        else
        {
            isMyGradientMagSmall = false;
        }

        //Asynchronous version, update the estimation error threshold based on the gradient magnitude
       if(!isFixedError)
       {
           this.estimationErrorThreshold = K4 * gradientMagnitude;
       }

        //Try to decrease the speed of robot when the gradient is smaller.
        if (gradientMagnitude < norminalGradientMagnitude)
        {
            speed = robotMaxSpeed * gradientMagnitude / norminalGradientMagnitude;
        }
        else
        {
            speed = robotMaxSpeed;
        }

        heading = Math.atan2(dFdy, dFdx); //final robot moving direction implementation
        if (heading == 0) //TODO Why heading will be zero?
        {
            speed = 0;
        }

        // System.out.println("heading:"+heading);
        //end vehicle handling
    }

    double SensingModelFunction(double distance) //exponentially decreasing with the distance
    {
        //Make sure it is between 0 to 1
        return Math.exp( -sensingDecayFactor * distance);
        //return 1-0.005*distance;

    }

    double SensingModelDerivative(double distance)
    {
        return -sensingDecayFactor * Math.exp( -sensingDecayFactor * distance);
        //return -0.005;
        //return 1/(distance+1) ;
    }
    
    synchronized void InformationValue()
    {
    	double informationValue = 0;
	   double temp;

       double delta = surface_integral_increment; //0.25 is our original value
       point2 samplePoint = new point2( -1, -1);
       double neighborEffects;
       double dist;
       double alpha;
        double sample_upper_bound_x = Math.min(position.x+sensingCutoffRange, boundary.largestX);
        double sample_upper_bound_y = Math.min(position.y+sensingCutoffRange, boundary.largestY);
        
        for (double horizontalSamplePoint =  Math.max(position.x-sensingCutoffRange, 0.01);horizontalSamplePoint<=sample_upper_bound_x; horizontalSamplePoint+=delta)
        {
            for (double verticalSamplePoint = Math.max(position.y-sensingCutoffRange, 0.01); verticalSamplePoint<=sample_upper_bound_y; verticalSamplePoint+=delta)
            {
                samplePoint.x = horizontalSamplePoint;
                samplePoint.y = verticalSamplePoint;
                dist = point2.Dist(samplePoint, position);
                      
                if(is_point_visible(samplePoint))
                {
                	alpha = 1;
                }
                else
                {
                	alpha = EventDensity.WALL_DECAY_FACTOR;
                }


                informationValue += alpha * NeighborEffects(samplePoint)* SensingModelFunction(point2.Dist(samplePoint, position));
            }
        }
        his_informationValue  =informationValue; //missing probability by neighbors
    }

    synchronized double NeighborEffects(point2 samplePoint)
    {
        double neighborEffect = 1;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = EventDensity.WALL_DECAY_FACTOR;
        	}
        	
        	neighborEffect *=  
        		(1 - alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).estimatedPosition)));
        }

        return neighborEffect; //missing probability by neighbors
    }

    public double detectionProbabilityReward(double prob)
    {
        assert (prob >= 0 && prob <= 1);

        if (prob < 0.1)
        {
            return prob * this.ultraLowDetectionBoost;
        }
        else if (prob < 0.5)
        {
            return (prob - 0.1) * this.smallDetectionBoost + 0.1 * this.ultraLowDetectionBoost;
        }
        else
        {
            return (prob - 0.5) + 0.4 * this.smallDetectionBoost + 0.1 * this.ultraLowDetectionBoost;
        }
    }

    //calculates data collection reward
    public double monitoringModelFunction(point2 eventLocation)
    {
        return Math.exp(-dataCollectionDecay * point2.Dist(eventLocation, this.position));
    }

    public double monitoringModelDerivative(Event e)
   {
       double currectMonitoringPercentage = 0;

       for(int i = 0; i < sim.realTimeRobotNumber; i++)
       {
           currectMonitoringPercentage += sim.robotList.get(i).monitoringModelFunction(e.eventLocation);
       }

      e.monitorPercentage = currectMonitoringPercentage;

      if(currectMonitoringPercentage>=1)
      {
          return 0;
      }
      else
      {
          return -dataCollectionDecay * Math.exp( -dataCollectionDecay * point2.Dist(e.eventLocation, this.position));
      }
   }

    public void clearTrajectoryHistory()
    {
        trajectoryHistoryIndex = 0;
    }
    
    public void move_to_robot(Robot seeking_target, double range)
    {
    	this.is_seeker_mode = true;
    	this.seeker_target = seeking_target;
    	this.seeking_hit_range = range;    
    }
    
    public void set_FOV_edge_points()
    {
    	this.FOV_edge_point_left.set(position.x+sensingCutoffRange*Math.cos(FOV_edge_heading_left), position.y+sensingCutoffRange*Math.sin(FOV_edge_heading_left));
    	this.FOV_edge_point_right.set(position.x+sensingCutoffRange*Math.cos(FOV_edge_heading_right), position.y+sensingCutoffRange*Math.sin(FOV_edge_heading_right));
    }
    
//    public void shift_sensor_heading(double dFdh)  //for smoothly adjust sensor heading
//    {
//    	double max_sensor_heading_turning_speed = 0.02; //was 0.01
//    	if(dFdh>0)
//    	{
//    		this.sensor_heading -= Math.min(max_sensor_heading_turning_speed,dFdh/100);
//    	}
//    	else if(dFdh<0)
//    	{
//    		this.sensor_heading += Math.min(max_sensor_heading_turning_speed,-dFdh/100);
//    	}
//    	
//    	FOV_edge_heading_left = sensor_heading-FOV_degree/2;
//    	FOV_edge_heading_right = sensor_heading+FOV_degree/2;;
//    	set_FOV_edge_points();
//    }
    
    public void modify_sensor_heading(double delta)  //for quickly adjust sensor heading
    { 
    	this.sensor_heading += delta;
 
    	FOV_edge_heading_left = sensor_heading-FOV_degree/2;
    	FOV_edge_heading_right = sensor_heading+FOV_degree/2;;
    	set_FOV_edge_points();
    }
    
    public void SetSensorHeading(double newHeading)  //for quickly adjust sensor heading
    { 
    	newHeading = newHeading%(2*Math.PI);
    	this.sensor_heading = newHeading;
 
    	FOV_edge_heading_left = sensor_heading-FOV_degree/2;
    	FOV_edge_heading_right = sensor_heading+FOV_degree/2;;
    	set_FOV_edge_points();
    }
    
    public void FastSearchSensorHeading()
    {
    	double originalSensorHeading = this.sensor_heading;
    	double bestSensorHeading = this.sensor_heading;
    	double bestObjValue = this.sim.EvaluateObj();
    	
    	for(int i = 1; i<4; i++)
    	{
	    	this.SetSensorHeading(originalSensorHeading+i*Math.PI/2);
	    	double newObjValue = this.sim.EvaluateObj();
	    	if (newObjValue>bestObjValue)
	    	{
	    		bestSensorHeading = originalSensorHeading+i*Math.PI/2;
	    		bestObjValue=newObjValue;	
	    	}
    	}
    	this.SetSensorHeading(bestSensorHeading);
    }
    //TODO: implement these
//    public boolean equals(Robot r)
//    {
//    	
//    }
//    
//    public int hashCode()
//    {
//    	int hash 9;
//    	hash = (31*hash)+id;
 //   	hash = (31*hash)+(null==position?0:position.hashCode());
//    	return hash;
//    }
    

}
