package coverage;

import java.awt.Toolkit;
import java.util.ArrayList;

import javax.swing.ProgressMonitor;
import javax.swing.SwingWorker;


public class StochasticComparison {
	
	public static long AngleIterationNum = 1000;
	
	 	public StochasticComparison()
	    {
		 
	    }

	 	public static void ImproveRobotStates(Simulation sim)
	 	{
	 		String originalButtonText = sim.coverage.stochasticComparisonButton.getText();
	 		ArrayList<point2> bestPositions = new ArrayList<point2>();
	 		ArrayList<Double> bestHeadings = new ArrayList<Double>();
	 		for(int j = 0; j<sim.robotList.size();j++)
 			{
	 			bestPositions.add(sim.robotList.get(j).position.Clone());
	 			bestHeadings.add(sim.robotList.get(j).sensor_heading);
 			}
	 		
	 		double bestObjValue = sim.EvaluateObj();
	 		System.out.println("Initial obj value:"+bestObjValue);
	 		
	 		for(int i=0;i<AngleIterationNum;i++)
	 		{
	 			sim.coverage.stochasticComparisonButton.setText("   "+i*100/AngleIterationNum+"%  ");
	 			for(int j = 0; j<sim.robotList.size();j++)
	 			{
	 				sim.robotList.get(j).position.x = Math.random()*(sim.boundary.largestX);
	 				sim.robotList.get(j).position.y = Math.random()*(sim.boundary.largestY);
	 				sim.robotList.get(j).SetSensorHeading( Math.random()*2*Math.PI);
	 			}
	 			double tempObjValue = sim.EvaluateObj();
	 			//System.out.println("tempObjValue:"+tempObjValue);
	 			if (tempObjValue>bestObjValue)
	 			{
	 				bestObjValue = tempObjValue;
	 				for(int j = 0; j<sim.robotList.size();j++)
		 			{
	 					bestPositions.get(j).x = sim.robotList.get(j).position.x;
	 					bestPositions.get(j).y = sim.robotList.get(j).position.y;
	 					bestHeadings.set(j,sim.robotList.get(j).sensor_heading);
		 			}
	 			}
	 		}
	 		
	 		System.out.println("Final obj value:"+bestObjValue);
	 		for(int j = 0; j<sim.robotList.size();j++)
 			{
	 			sim.robotList.get(j).position.x = bestPositions.get(j).x;
 				sim.robotList.get(j).position.y = bestPositions.get(j).y;
	 			sim.robotList.get(j).SetSensorHeading(bestHeadings.get(j));
 			}
	 		sim.coverage.stochasticComparisonButton.setText(originalButtonText);
	 	}
	 	
	 	public static void ImproveRobotAngles(Simulation sim)
	 	{
	 		ArrayList<Double> bestHeadings = new ArrayList<Double>();
	 		for(int j = 0; j<sim.robotList.size();j++)
 			{
	 			bestHeadings.add(sim.robotList.get(j).sensor_heading);
 			}
	 		
	 		double bestObjValue = sim.EvaluateObj();
	 		System.out.println("Initial obj value:"+bestObjValue);
	 		
	 		for(int i=0;i<AngleIterationNum;i++)
	 		{
	 			for(int j = 0; j<sim.robotList.size();j++)
	 			{
	 				sim.robotList.get(j).SetSensorHeading( Math.random()*2*Math.PI);
	 			}
	 			double tempObjValue = sim.EvaluateObj();
	 			//System.out.println("tempObjValue:"+tempObjValue);
	 			if (tempObjValue>bestObjValue)
	 			{
	 				bestObjValue = tempObjValue;
	 				for(int j = 0; j<sim.robotList.size();j++)
		 			{
	 					bestHeadings.set(j,sim.robotList.get(j).sensor_heading);
		 			}
	 			}
	 		}
	 		
	 		System.out.println("Final obj value:"+bestObjValue);
	 		for(int j = 0; j<sim.robotList.size();j++)
 			{
	 			sim.robotList.get(j).SetSensorHeading(bestHeadings.get(j));
 			}
	 	}
}

class StochasticComparisonTask extends SwingWorker<Void, Void> {
	Coverage coverage;
	
	public StochasticComparisonTask(Coverage c)
	{
		coverage = c;
	}
    @Override
    public Void doInBackground() {
    	coverage.graphPanel.stop();
    	if(coverage.start_demo)
    	{
    		coverage.start_demo = false;
        	StochasticComparison.ImproveRobotStates(coverage.sim);
        	//StochasticComparison.ImproveRobotAngles(sim);
        	coverage.start_demo = true;
    	}
    	else
    	{
    		StochasticComparison.ImproveRobotStates(coverage.sim);
    		coverage.graphPanel.read_nodes_pos();
    		coverage.updateTable();
    	}
    	coverage.graphPanel.start();        
    	return null;
    }

    @Override
    public void done() {
        Toolkit.getDefaultToolkit().beep();

    }
}


