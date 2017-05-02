package coverage;

import java.util.ArrayList;

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
public class EventDensity
{
	double _maxX;
	double _maxY;
	double _minX;
	double _minY;
	
	//event generation stuff
	double lastEventTime = 0;

	//grid representation of the true event density. The origin of each grid at the upper-left corner
	static double GRID_SIZE = 1;
	int numberRows, numberColumns;  //determined when the dimension of the rectangular boundary is determined.
	double gridEventDensity[][];
	double rowSumEventDensity[];   //sum the density of each row into a vector. For event generation
	double totalSumEventDensity = 0;

	//Gaussian peak related, now used for target tracking
	public boolean peak_in_effect = false;
	public point2 peak= new point2(-1,-1); 
	
	//moving data source
	public ArrayList<DataSource> MovingDataSources = new ArrayList<DataSource>();
	public int MaxDataSourceNum = 25; 
	public int ActiveDataSourceNum = 5;

	public EventDensity(double maxX, double maxY, double minX, double minY)
	{
		_maxX = maxX;
		_maxY = maxY;
		_minX = minX;
		_minY = minY;
		
		for(int i = 0; i<MaxDataSourceNum ; i++)
		{
			MovingDataSources.add(new DataSource(Math.random()*(maxX-minX),Math.random()*(maxY-minY),10000.0,Math.random()*2*Math.PI,3.0));
		}
	}
	
	public void UpdateDataSourceStates(double simulationTime, ArrayList<Obstacle> obstacles, ArrayList<Robot> robots, int realTimeRobotNumber)
	{
		for(int i = 0; i<ActiveDataSourceNum; i++)
		{
			MovingDataSources.get(i).UpdateState(simulationTime,_maxX,_maxY,_minX, _minY, obstacles,robots, realTimeRobotNumber);
		}
	}

	public void initTrueEventDensity(double width, double height)
	{
		point2 currentGridOrigin = new point2(0,0); //start from (0,0)
		numberRows = (int)Math.round(height / GRID_SIZE);
		numberColumns =(int)Math.round( width / GRID_SIZE);
		gridEventDensity = new double[numberRows][numberColumns];
		rowSumEventDensity = new double[numberRows];
		// initialize the density with a nominal number, currently set to be 1
		for(int i = 0; i < numberRows; i++)
		{
			for(int j = 0; j < numberColumns; j ++)
			{
				gridEventDensity[i][j] = 1;
			}
		}

		//set density other than the nominal number
		//make the lower left higher
		for(int i = numberRows/2; i < numberRows; i++)
		{
			for(int j = numberColumns/2; j < numberColumns; j ++)
			{
				gridEventDensity[i][j] = 10;
			}
		}

		//find the sum of each row and total sum
		for(int i = 0; i < numberRows; i++)
		{
			rowSumEventDensity[i] = 0;
			for(int j = 0; j < numberColumns; j ++)
			{
				rowSumEventDensity[i] += gridEventDensity[i][j];
			}
			totalSumEventDensity += rowSumEventDensity[i];
		}
	}

	// if a new event is generated, return true
	public boolean generateEvent(Event newEvent, double time)
	{
		if(time - lastEventTime>=Event.EVENT_SPAWN_INTERVAL)
		{
			//find the new event location based on the hidden true event density
			//first determine which grid is selected; then randomly select one point in the grid
			//first find which row is selected; then which column in that row
			double randomNumber = Math.random();
			int selectedRow;
			int selectedColumn;
			double currentSum;
			for(selectedRow = 0,currentSum =rowSumEventDensity[0] ; selectedRow < numberRows; selectedRow++,currentSum+=rowSumEventDensity[selectedRow])
			{
				if(randomNumber <= currentSum/totalSumEventDensity) //currentSum/totalSumEventDensity will reach 1 finally, so this test is guaranteed to be true once.
				{
					break;  //the current selectedRow is the selected row
				}
			}

			randomNumber = Math.random();
			for(selectedColumn = 0,currentSum =gridEventDensity[selectedRow][0] ; selectedColumn < numberColumns; selectedColumn++,currentSum+=gridEventDensity[selectedRow][selectedColumn])
			{
				if(randomNumber <= currentSum/rowSumEventDensity[selectedRow]) //currentSum/totalSumEventDensity will reach 1 finally, so this test is guaranteed to be true once.
				{
					break;  //the currect selectedColumn is the selected column
				}

			}

			newEvent.eventLocation.x = GRID_SIZE*selectedColumn+Math.random()*GRID_SIZE;
			newEvent.eventLocation.y = GRID_SIZE*selectedRow+Math.random()*GRID_SIZE;
			newEvent.spawnTime = time;
			lastEventTime = time;
			return true;
		}
		else
		{
			return false   ;
		}
	}

	double GetEventDensity(point2 queryPosition, double currentTime)
	{
		double dataSourceDensity = 0;
		for(int i=0;i<ActiveDataSourceNum;i++)
		{
			DataSource ds = MovingDataSources.get(i);
			//dataSourceDensity += MovingDataSources.get(i).Intensity*Math.exp(-point2.Dist(queryPosition, MovingDataSources.get(i))/MovingDataSources.get(i).GaussianVariance);
			for(int j=0;j<ds.TraceLocations.size();j++)
			{
				dataSourceDensity += ds.TraceLocations.get(j).Intensity*Math.exp(-point2.Dist(queryPosition, ds.TraceLocations.get(j))/ds.GaussianVariance);
			}
		}

		return 1+dataSourceDensity;

		//a Gaussian at (30,25)
		//double xVariance = 0.1;
		//   double yVariance = 0.1;
		//  return  1+100000*Math.exp(-Math.pow((queryPosition.x - 30),2)/xVariance-Math.pow((queryPosition.y - 25),2)/yVariance);

//		if(peak_in_effect)
//		{
//			//a Gaussian at (peak.x,peak.y)
//			double xVariance = 1;
//			double yVariance = 1;
//			return  1+200000*Math.exp(-Math.pow((queryPosition.x - peak.x),2)/xVariance-Math.pow((queryPosition.y - peak.y),2)/yVariance);
//		}
//		else
//		{
//			return 1;
//		}

	}

	double GetEventDensity(point2 queryPosition)
	{
		return GetEventDensity(queryPosition,-1);
	}

	public static double WALL_DECAY_FACTOR = 0; //determines how much reward a sensor gets when a location is blocked by obstacle or out of its FOV
}

class DataSource extends point2
{
	public double GaussianVariance = 0.12; //was 0.2 smaller value = narrower peak
	public double Heading = 0;
	public double Speed = 1;
	private double _previousDataSourceUpdateTime = 0;
	private int _noMoveCount = 0;  //data source gets stuck in an obstacle if it does not move for a while
	private int _maxNoMoveCount = 500;
	public ArrayList<point2> TraceLocations = new ArrayList<point2>();
	public int MaxTraceLocation = 3; //5 is the old value
	private int _tracingInterval = 80;
	private int _tracingIntervalCount = 70;
	
	public DataSource(double x, double y, double intensity, double h, double sp)
	{
		super(x,y,intensity);
		Heading = h;
		Speed = sp;
		
//		for(int i=0; i<MaxTraceLocation;i++)
//		{
//			TraceLocations.add(new point2(x,y,0));
//		}
	}
	
	public void UpdateState(double simulationTime, double maxX, double maxY, double minX, double minY,ArrayList<Obstacle> obstacles,ArrayList<Robot> robots, int realTimeRobotNumber)
	{
		if(_noMoveCount>_maxNoMoveCount)
		{
			x = Math.random()*(maxX-minX);
			y = Math.random()*(maxY-minY);
			_noMoveCount = 0;
		}
		
		double timeInterval = simulationTime - _previousDataSourceUpdateTime;
		_previousDataSourceUpdateTime = simulationTime;
		double oldX = x;
		double oldY = y;
		Move(timeInterval);
		if(x<=minX || x>=maxX ||y<=minY || y>=maxY) 
		{
			_noMoveCount++;
			x=oldX;
			y=oldY;
			Heading = Math.random()*Math.PI*2;
			return;
		}
		for(int i=0;i<obstacles.size();i++)
		{
			if(obstacles.get(i).IsInteriorPoint(this))
			{
				_noMoveCount++;
				x=oldX;
				y=oldY;
				Heading = Math.random()*Math.PI*2;
				return;
			}
		}
	
		
		//UpdateTrace();
		UpdateOccupancyGrid(robots,realTimeRobotNumber);
		ChangeIntensity(simulationTime);
	}
	
	private void UpdateOccupancyGrid(ArrayList<Robot> robots, int realTimeRobotNumber)
	{
		boolean alreadyInTrace = false;
		point2 dataSourceGridLocation = new point2(1.0 + 2 * Math.floor(this.x / 2), 1.0 + 2 * Math.floor(this.y / 2), 1000.0);
		for (int i = TraceLocations.size() - 1; i >= 0; i--) 
		{
			if (point2.equal(TraceLocations.get(i), dataSourceGridLocation)) 
			{
				alreadyInTrace = true;
			}
			
			boolean isInDetectionRange = false;
			for (int i1 = 0; i1 < realTimeRobotNumber; i1++) 
			{
				if (robots.get(i1).is_point_visible(TraceLocations.get(i))) 
				{
					isInDetectionRange = true;
					break;
				}
			}
			if (isInDetectionRange) 
			{
				if (point2.equal(TraceLocations.get(i), dataSourceGridLocation)) 
				{
					if (TraceLocations.get(i).Intensity < 500000.0) 
					{
						TraceLocations.get(i).Intensity *= 1.5;
					}
//					else
//					{
//						System.out.println("TraceLocations.get(i).Intensity="+TraceLocations.get(i).Intensity);
//					}
				} 
				else 
				{
					TraceLocations.get(i).Intensity /= 1.3;
					if (TraceLocations.get(i).Intensity < 100.0) 
					{
						TraceLocations.remove(i);
					}
				}
			}
		}
		if (!alreadyInTrace) {
			boolean isInDetectionRange = false;
			for (int i1 = 0; i1 < realTimeRobotNumber; i1++) 
			{
				if (robots.get(i1).is_point_visible(this)) 
				{
					isInDetectionRange = true;
					break;
				}
			}
			if (isInDetectionRange)
			TraceLocations.add(dataSourceGridLocation);
		}
	}
	
	private void UpdateTrace()
	{
		_tracingIntervalCount++;
		if(_tracingIntervalCount>=_tracingInterval)
		{
			for(int i = 0; i<MaxTraceLocation-1;i++)
			{
				TraceLocations.get(i).copy(TraceLocations.get(i+1));
				TraceLocations.get(i).Intensity /= 2;
			}
			_tracingIntervalCount = 0;
		}
		
		TraceLocations.get(MaxTraceLocation-1).copy(this);
	}
	
	private void Move(double timeInterval)
	{
		x = x+Speed*timeInterval*Math.cos(Heading);
		y = y+Speed*timeInterval*Math.sin(Heading);	
	}
	
	private void ChangeIntensity(double simulationTime)
	{
	
	}
}
