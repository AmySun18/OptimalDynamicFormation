package coverage;
import java.util.ArrayList;
import java.awt.*;

public class MapBuildingGraphPanel extends GraphPanel {

	public MapBuildingGraphPanel(Coverage cvrg) {
		super(cvrg);
	}
	
	
	 protected void paintNode(Graphics g, Node n, FontMetrics fm)
	    {
		 	super.paintNode(g, n, fm);
		 
	      //draw the sensing radius
	      g.setColor(Color.gray);
	      int x = (int) n.x;
	      int y = (int) n.y;
	      int discoverageRange = (int) coverage.sim.rangefinderRange; 
	      //drawOval seems to be a performance hit
	      g.drawOval(x - discoverageRange * DISPLAY_MULTIPLIER, y - discoverageRange * DISPLAY_MULTIPLIER, 2 * discoverageRange * DISPLAY_MULTIPLIER,
	                 2 * discoverageRange * DISPLAY_MULTIPLIER);

	        

	    }
	
	protected synchronized void paintScreen()
    {
        //super.paintComponent(g);

        Dimension d = getSize();
        if ((offscreen == null) || (d.width != offscreensize.width) || (d.height != offscreensize.height))
        {
            offscreen = createImage(d.width, d.height);
            offscreensize = d;
            if (offgraphics != null)
            {
                offgraphics.dispose();
            }
            //offgraphics = offscreen.getGraphics(); //not such a big difference with Graphics2D
            offgraphics = (Graphics2D) offscreen.getGraphics();
            offgraphics.setFont(getFont());
        }

        //redraw the background of the whole graph area
        //offgraphics.setColor(getBackground());
        offgraphics.setColor(Color.white);
        offgraphics.fillRect(0, 0, d.width, d.height);

        FontMetrics fm = offgraphics.getFontMetrics();

        //draw tick
        offgraphics.setColor(Color.black);
        for (int i4 = 0; i4 <= coverage.world_size_x; i4 = i4 + 5)
        {
            offgraphics.drawString(Integer.toString(i4), (int) zoomIn(i4) - 4, DRAWING_OFFSET - fm.getHeight() + 12);
        }
        for (int i4 = 0; i4 <= coverage.world_size_y; i4 = i4 + 5)
        {
            offgraphics.drawString(Integer.toString(i4), DRAWING_OFFSET - fm.stringWidth(Integer.toString(i4)) - 3, (int) zoomIn(i4) + 4);
        }

        //draw coverage map
        if (this.coverage.evaluateObjFunc)
        {
            double inc = coverage.sim.objEvalIncrement;
            float prob;
            int interval = (int) (inc * DISPLAY_MULTIPLIER);
            try
            {
                for (int i = 0; i < this.coverage.sim.coverageMap.length; i++)
                {
                    for (int j = 0; j < coverage.sim.coverageMap[i].length; j++)
                    {
                        prob = (float) coverage.sim.coverageMap[i][j];

                        if (prob > 0.5)
                        {
                            offgraphics.setColor(new Color(1f, 1f, 2f * (prob - 0.5f)));
                        }
                        else if (prob > 0.03)
                        {
                            offgraphics.setColor(new Color(0f, 1f, prob * 2));
                        }
                        else
                        {
                            //offgraphics.setColor(new Color(50,150,50));
                            offgraphics.setColor(Color.magenta);
                        }
                        offgraphics.fillRect((int) zoomIn(j * inc), (int) zoomIn(i * inc), interval, interval);
                    }
                }
            } catch (ArrayIndexOutOfBoundsException e)
            {
            }
        }

        //draw boundary
        offgraphics.setColor(Color.black);
        offgraphics.drawRect(DRAWING_OFFSET, DRAWING_OFFSET, coverage.world_size_x * DISPLAY_MULTIPLIER,
                             coverage.world_size_y * DISPLAY_MULTIPLIER);
        //draw obstacles
        offgraphics.setColor(Color.blue);    
        try
        {       	
            for (int i3 = 0; i3 < coverage.obstacle_number; i3++)
            {
            	// obstaclesForDrawing[obstacle_load_index].addPoint((int) GraphPanel.zoomIn(Double.parseDouble(splitString[i * 2])),
                  //       (int) GraphPanel.zoomIn(Double.parseDouble(splitString[i * 2 + 1])));
            	
            	//if(coverage.sim.obstacles.get(i3).isDiscovered)        	
            	this.offgraphics.drawPolygon(coverage.obstaclesForDrawing[i3]);     	
            }
            
            offgraphics.setStroke (thickStroke); 
            for (int i3 = 0; i3 < coverage.obstacle_number; i3++)
            {    
            	ArrayList<point2> vt = coverage.sim.obstacles.get(i3).vertices;
            	point2 previousPoint = vt.get(vt.size() - 1);
                for (int i = 0; i < vt.size(); i++)
                {
                	if (previousPoint.isDiscovered && vt.get(i).isDiscovered)
                    {
                		this.offgraphics.drawLine((int)zoomIn(vt.get(i).x), (int)zoomIn(vt.get(i).y), (int)zoomIn(previousPoint.x), (int)zoomIn(previousPoint.y));
                    }
                    previousPoint = vt.get(i);
                }
 	
            }
        } catch (NullPointerException e)
        {
        	e.printStackTrace();
        }
        catch (IndexOutOfBoundsException e)
        {
        	e.printStackTrace();
        	System.out.println("coverage.obstacle_number:"+coverage.obstacle_number);
        	System.out.println("coverage.sim.obstacles.size():"+coverage.sim.obstacles.size());
        }
        offgraphics.setStroke (thinStroke);
        
        //draw moving targets (Gaussian peaks of event density)
//        {
//        	if(coverage.sim.target1.trajectoryStarted&& !coverage.sim.target1.trajectoryEnded)
//        	{
//        		offgraphics.setColor(new Color(178,0,255));
//        		offgraphics.fillOval((int) zoomIn(coverage.sim.target1.x) - 20,
//                        (int) zoomIn(coverage.sim.target1.y) - 20, 40, 40);
//        	}
//        }
        
        //draw map building progress
        if (this.coverage.is_obstacle_discovery_mode)
        {
        	double inc = coverage.sim.mapBuildingIncrement;
        	//int interval = (int) (inc * DISPLAY_MULTIPLIER);
        	offgraphics.setColor(Color.red);
        	try
        	{
        		for (int i = 0; i < this.coverage.sim.mapBuildingGrid.length; i++)
        		{
        			for (int j = 0; j < coverage.sim.mapBuildingGrid[i].length; j++)
        			{
        				if (coverage.sim.mapBuildingGrid[i][j] == 1)
        				{
        					offgraphics.drawOval( (int) zoomIn((0.5+i) * inc),(int) zoomIn((j+0.5) * inc), 1, 1);
        				}
        				else if (coverage.sim.mapBuildingGrid[i][j] == 3)
        				{
        					offgraphics.fillOval( (int) zoomIn((0.5+i) * inc),(int) zoomIn((j+0.5) * inc), 5, 5);
        				}      				
        			}
        		}
        	} catch (ArrayIndexOutOfBoundsException e)
        	{
        	}
        }
        
        //draw robots

        //paint trajectory
        if (coverage.showTrajectory)
        {
        	for (int i8 = 0; i8 < coverage.sim.realTimeRobotNumber; i8++)
        	{
        		int index = coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryIndex;
        		if (index > 1)
        		{
        			offgraphics.setColor(Color.black);
        			for (int i = 1; i < index; i++)
        			{
        				offgraphics.drawLine((int) zoomIn(coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryX[i]),
        						(int) zoomIn(coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryY[i]),
        						(int) zoomIn(coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryX[i + 1]),
        						(int) zoomIn(coverage.sim.robotList.get(nodes[i8].id).trajectoryHistoryY[i + 1]));
        			}
        		}
        	}
        }

        //just draw the number of nodes user want
        for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
        {
        	paintNode(offgraphics, nodes[i], fm);

        }

        //draw communication link, disabled temperarily
        /*  offgraphics.setColor(Color.BLACK);
            for (int i = 0; i < coverage.sim.realTimeRobotNumber; i++)
            {
            	offgraphics.drawLine((int) zoomIn(coverage.sim.robotList.get(i).position.x),
            			(int) zoomIn(coverage.sim.robotList.get(i).position.y),
            			(int) zoomIn(coverage.sim.robotList.get(i).predecessor.position.x),
            			(int) zoomIn(coverage.sim.robotList.get(i).predecessor.position.y));
            	offgraphics.drawString(Float.toString((float) coverage.sim.robotList.get(i).costToBase),
            			(int) zoomIn((coverage.sim.robotList.get(i).position.x)),
            			(int) zoomIn((coverage.sim.robotList.get(i).position.y - 0.6)));
            }*/

        //draw active events
       offgraphics.setColor(Color.red);  //undetect events should be draw with another color
       for(int i = 0; i<coverage.sim.currentDetectedEvents.size(); i++)
       {
           Event currentEvent = coverage.sim.currentDetectedEvents.get(i);
           drawStarShape(currentEvent.eventLocation);
           offgraphics.drawString(Integer.toString((int)(100.0*currentEvent.monitorPercentage)), (int)zoomIn(currentEvent.eventLocation.x) - 6, (int)zoomIn(currentEvent.eventLocation.y) - 6 );
       }

        //draw group selection rectangle
        if (InGroupSelection)
        {
            offgraphics.setColor(Color.gray);
            this.offgraphics.drawRect(groupSelectionAnchorX, groupSelectionAnchorY, groupSelectionW, groupSelectionH);
        }

        Graphics g;
        try
        {
            g = this.getGraphics();
            if (g != null)
            {
                g.drawImage(offscreen, 0, 0, null);
            }
            Toolkit.getDefaultToolkit().sync(); // sync the display on some systems
            g.dispose();
        } catch (Exception e) // quite commonly seen at applet destruction
        {
            System.out.println("Graphics Context error: " + e);
        }

    }

}
