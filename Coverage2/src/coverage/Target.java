package coverage;

import java.util.ArrayList;

public class Target {
	public Target(){
		
		double current_trajectory_time = 5;
		double time_between_points = 15;
		
		trajectory = new ArrayList<point2>();
		trajectoryTimeline = new ArrayList<Double>();
		
		//target trajectory is specified here
		for(int i = 0; i<10;i++)  //repeat for ten circles
		{
		trajectory.add(new point2(1,5));
		trajectoryTimeline.add(current_trajectory_time+=time_between_points);

		trajectory.add(new point2(59,5));
		trajectoryTimeline.add(current_trajectory_time+=time_between_points);
		
		trajectory.add(new point2(55,49));
		trajectoryTimeline.add(current_trajectory_time+=time_between_points);
		
		trajectory.add(new point2(21,46));
		trajectoryTimeline.add(current_trajectory_time+=time_between_points/2);
		
		trajectory.add(new point2(21,15));
		trajectoryTimeline.add(current_trajectory_time+=time_between_points/2);
		
		trajectory.add(new point2(1,1));
		trajectoryTimeline.add(current_trajectory_time+=time_between_points/2);
		
		trajectory.add(new point2(5,45));
		trajectoryTimeline.add(current_trajectory_time+=time_between_points);
		
		trajectory.add(new point2(5,49));
		trajectoryTimeline.add(current_trajectory_time+=time_between_points/2);
		}
		
	};
	public double x = -1;
	public double y = -1;
	public ArrayList<point2> trajectory;
	public ArrayList<Double> trajectoryTimeline; //one time one point2

	public int current_trajectory_point = -1;
	public boolean trajectoryStarted = false;
	public boolean trajectoryEnded = false;
	
	private double local_time = 0;
	private double last_global_time = 0;

	public void targetLocationUpdate(double global_time)
	{
		double time_increment = global_time - last_global_time;
		last_global_time = global_time;
		local_time += time_increment;
		if(!trajectoryEnded)
		{
			if(!trajectoryStarted)
			{
				if(local_time>=trajectoryTimeline.get(0))
				{
					trajectoryStarted = true;
					current_trajectory_point = 0; //we are starting to move from point 0
					x = trajectory.get(0).x;
					y = trajectory.get(0).y;
				}
			}
			else
			{
				if(local_time>=trajectoryTimeline.get(current_trajectory_point+1))
				{
					x = trajectory.get(current_trajectory_point+1).x;
					y = trajectory.get(current_trajectory_point+1).y;
					if(current_trajectory_point+2<trajectoryTimeline.size())
					{
						current_trajectory_point = current_trajectory_point+1; //move towards the next waypoint
					}
					else
					{
						//if you want the target to circle forever, use the following lines
						//current_trajectory_point = 0;
						//local_time = 0;
						//if you want the target to circle only once, use the following line
						trajectoryEnded = true;
					}	
				}
				else //move along the current trajectory
				{
					double time_portion_passed = (local_time-trajectoryTimeline.get(current_trajectory_point))
					/(trajectoryTimeline.get(current_trajectory_point+1)-trajectoryTimeline.get(current_trajectory_point));
					x = time_portion_passed*trajectory.get(current_trajectory_point+1).x
					+(1-time_portion_passed)*trajectory.get(current_trajectory_point).x;
					y =  time_portion_passed*trajectory.get(current_trajectory_point+1).y
					+(1-time_portion_passed)*trajectory.get(current_trajectory_point).y;
				}
			}
		}

	}
}
