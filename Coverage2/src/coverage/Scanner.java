package coverage;

import java.applet.Applet;
import java.util.ArrayList;
import java.io.InputStream;
import java.io.IOException;
import java.net.MalformedURLException;
import java.io.InputStreamReader;
import java.net.URL;
import java.io.BufferedReader;
import java.awt.Polygon;
import javax.swing.JTable;
import javax.swing.JTextArea;
import javax.swing.table.TableColumn;
import java.awt.BorderLayout;
import javax.swing.JScrollPane;
import javax.swing.SpinnerNumberModel;
import javax.swing.BorderFactory;
import javax.swing.JPanel;
import javax.swing.JLabel;
import javax.swing.JSpinner;
import java.awt.Dimension;
import java.awt.Color;
import javax.swing.JButton;
import javax.swing.JComboBox;
import java.awt.event.ActionEvent;
import java.awt.event.ItemEvent;

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
public class Scanner extends Coverage
{
     JButton scan = new JButton("     Scan    ");
     JComboBox scanIntervalCombobox = new JComboBox();
    //JButton saveScanResult = new JButton("     Save    ");

    public Scanner()
    {
    }
    public void init()
    {
        LoadEverything();

        //initialize some simulation related data structures
        density = new EventDensity(boundary.largestX,boundary.largestY,boundary.smallestX,boundary.smallestY);
        sim = new Simulation(boundary, obstacles, density, 4, 6, new ArrayList<point2>(), 15, this,InitialSensorDecay); //4 is the current starting number of robots       
        MakGUILayout();
    }

    public void destroy()
   {
       remove(graphPanel);
   }

   public void start()
   {
        graphPanel.start();
   }

   public void stop()
   {

       graphPanel.stop();
   }

   private void MakGUILayout()
    {
        setLayout(new BorderLayout());

       graphPanel = new GraphPanel(this);
       add("Center", graphPanel);

       //North Panel
       JPanel northPanel = new JPanel();
       add("North", northPanel);

       northPanel.add(scan);
       scan.addActionListener(this);

       // robot number control
     robotNumberSpinnerModel = new SpinnerNumberModel(this.sim.realTimeRobotNumber, 0, this.MAX_ROBOT_NUMBER, 1);
     this.realTimeRobotNumberSpinner = new JSpinner(this.robotNumberSpinnerModel);
     northPanel.add(new JLabel("Nodes#"));
     northPanel.add(this.realTimeRobotNumberSpinner);
     robotNumberSpinnerModel.addChangeListener(this);
     this.realTimeRobotNumberSpinner.setEnabled(true);

       insertVerticalSeparator(northPanel);

            northPanel.add(new JLabel("Objective Resolution")); //objective function integral resolution
            northPanel.add(objEvaluationIntervalCombobox);
            objEvaluationIntervalCombobox.addItem(0.25);
            objEvaluationIntervalCombobox.addItem(0.5);
            objEvaluationIntervalCombobox.addItem(1);
            objEvaluationIntervalCombobox.setSelectedIndex(2);
            objEvaluationIntervalCombobox.addItemListener(this);
       objEvaluationIntervalCombobox.setEnabled(true);

       northPanel.add(new JLabel("Scan Resolution")); //scan resolution
                 northPanel.add(scanIntervalCombobox);
               //  scanIntervalCombobox.addItem(0.25);
                 scanIntervalCombobox.addItem(0.5);
                 scanIntervalCombobox.addItem(1);
                 scanIntervalCombobox.addItem(2);
                 scanIntervalCombobox.setSelectedIndex(1);
                 scanIntervalCombobox.addItemListener(this);
       scanIntervalCombobox.setEnabled(true);

       //right panel
       rightPanel.setPreferredSize(new Dimension(225, 400));
       add("East", rightPanel);

       String[] columnNames =
               {"ID", "X", "Y", "Heading", "dH1x", "dH1y", "dH2x", "dH2y"};
       table_data = new Object[MAX_ROBOT_NUMBER][8];
       for (int i5 = 0; i5 < this.MAX_ROBOT_NUMBER; i5++)
       {

           this.table_data[i5][0] = i5;
           if (i5 < this.robots_init_pos_list.size())
           {
               table_data[i5][1] = this.robots_init_pos_list.get(i5).x;
               table_data[i5][2] = this.robots_init_pos_list.get(i5).y;
               table_data[i5][3] = 0;
           }
       }

       RobotTable = new JTable(table_data, columnNames);
       RobotTable.setPreferredScrollableViewportSize(new Dimension(200, 230));  //control the size of robot table here
       RobotTable.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
       RobotTable.setEnabled(false); //Do not let user edit this table
       TableColumn col = RobotTable.getColumnModel().getColumn(0);
       col.setPreferredWidth(25);
       for (int i5 = 1; i5 < columnNames.length; i5++)
       {
           col = RobotTable.getColumnModel().getColumn(i5);
           col.setPreferredWidth(55);
       }

       JScrollPane scrollPane = new JScrollPane(RobotTable);
       rightPanel.add("North", scrollPane);

       obstaclesInputTextfield = new JTextArea();
       obstaclesInputTextfield.setBorder(BorderFactory.createEtchedBorder());
       obstaclesInputTextfield.setText(obstaclesString);
       obstaclesInputTextfield.setTabSize(2);
       JScrollPane scrollPane1 = new JScrollPane(obstaclesInputTextfield);
       scrollPane1.setPreferredSize(new Dimension(100, 310)); //control the size of obstacle list here
       rightPanel.add("South", scrollPane1);

       updateObstaclesButton.setPreferredSize(new Dimension(30, 20));
       updateObstaclesButton.setMaximumSize(new Dimension(30, 20));
       rightPanel.add("Center", this.updateObstaclesButton);
       updateObstaclesButton.addActionListener(this);

       //Control panel
       //Control Parameters
       add("South", controlPanel);

       controlPanel.add(new JLabel("Integration Resolution"));
       controlPanel.add(this.surfaceIntegralIncrementTextfield);
       //  surfaceIntegralIncrementTextfield.setPreferredSize(new Dimension(40,21));
       surfaceIntegralIncrementTextfield.setText(Double.toString(sim.robotList.get(0).surface_integral_increment));
       surfaceIntegralIncrementTextfield.addActionListener(this);
       surfaceIntegralIncrementTextfield.setEnabled(true);

       controlPanel.add(new JLabel("Max Speed"));
       controlPanel.add(nodeSpeedTextfield);
       nodeSpeedTextfield.setText(Double.toString(sim.robotList.get(0).robotMaxSpeed));
       nodeSpeedTextfield.addActionListener(this);
       nodeSpeedTextfield.setEnabled(true);

       controlPanel.add(new JLabel("Sensing Decay"));
       controlPanel.add(sensingDecayTextfield);
       sensingDecayTextfield.setText(Double.toString(sim.robotList.get(0).sensingDecayFactor)+"   ");
       sensingDecayTextfield.addActionListener(this);
       sensingDecayTextfield.setEnabled(true);
       sensingDecayTextfield.setEditable(false);

       controlPanel.add(new JLabel("Sensing Range"));
       controlPanel.add(sensingCutoffRangeTextfield);
       sensingCutoffRangeTextfield.setText(Double.toString(sim.robotList.get(0).sensingCutoffRange));
       sensingCutoffRangeTextfield.addActionListener(this);
       sensingCutoffRangeTextfield.setEnabled(true);

       controlPanel.add(new JLabel("Max Norm"));
       controlPanel.add(this.nominalGradientMag);
       nominalGradientMag.setText(Double.toString(sim.robotList.get(0).norminalGradientMagnitude));
       nominalGradientMag.addActionListener(this);
       nominalGradientMag.setEnabled(true);

       controlPanel.add(new JLabel("Comm. Weight"));
       controlPanel.add(this.communicationWeightTextfield);
       communicationWeightTextfield.setText(Double.toString(sim.robotList.get(0).communicationCostWeight)+"      ");
       communicationWeightTextfield.addActionListener(this);

       controlPanel.add(new JLabel("Est. Threshold"));
      controlPanel.add(this.estimationErrorThresholdTextfield);
      estimationErrorThresholdTextfield.setText(Double.toString(sim.robotList.get(0).estimationErrorThreshold)+"      ");
      estimationErrorThresholdTextfield.addActionListener(this);

     //  insertVerticalSeparator(controlPanel);

    }

    private void LoadEverything() throws NumberFormatException
    {
        //read in parameters (file names) from html
        String world_datafile_name = getParameter("world_data");
        if (world_datafile_name == null)
        {
            world_datafile_name = "cluttered_world.txt"; //the default mission space if not specified in the html
        }

        //load data file
        String line;
        URL url2 = null;

        try
        {
            url2 = new URL(getCodeBase(), world_datafile_name);
        } catch (MalformedURLException e)
        {
            System.out.println("Malformed URL ");
            stop();
        }

        try
        {
            //load world geometry for display and simulation
            InputStream in = url2.openStream();
            BufferedReader dis = new BufferedReader(new InputStreamReader(in));
            String[] splitString;
            this.world_size_x = Integer.parseInt(dis.readLine());
            this.world_size_y = Integer.parseInt(dis.readLine());

            boundary = new Obstacle();
            boundary.vertices.add(new point2(0, 0));
            boundary.vertices.add(new point2(this.world_size_x, 0));
            boundary.vertices.add(new point2(this.world_size_x, this.world_size_y));
            boundary.vertices.add(new point2(0, this.world_size_y));
            boundary.updateBoundingBox();

            this.obstacle_number = Integer.parseInt(dis.readLine());
            obstaclesForDrawing = new Polygon[this.obstacle_number];
            int obstacle_load_index = 0;
            while ((line = dis.readLine()) != null)
            {
                line = line.replaceAll("[\t,;a-zA-Z]", " ");
               line = line.replaceAll(" +", " ");
                splitString = line.split(" ");
                if (splitString.length > 1)
                {
                    obstaclesString = obstaclesString + line.replace('\t', ' ') + '\n';
                    obstaclesForDrawing[obstacle_load_index] = new Polygon();
                    Obstacle newObstacle = new Obstacle();
                    for (int i = 0; i < splitString.length / 2; i++)
                    {
                        newObstacle.vertices.add(new point2(Double.parseDouble(splitString[i * 2]) + Math.random() * 0.00,
                                Double.parseDouble(splitString[i * 2 + 1]) + Math.random() * 0.00));

                        //obstaclesForDrawing[obstacle_load_index].addPoint(Integer.parseInt(splitString[i * 2]) * DISPLAY_MULTIPLIER +
                        //DRAWING_OFFSET, Integer.parseInt(splitString[i * 2 + 1]) * DISPLAY_MULTIPLIER + DRAWING_OFFSET);
                        obstaclesForDrawing[obstacle_load_index].addPoint((int) GraphPanel.zoomIn(Double.parseDouble(splitString[i * 2])),
                                (int) GraphPanel.zoomIn(Double.parseDouble(splitString[i * 2 + 1])));
                    }

                    obstacle_load_index++;

                    newObstacle.updateInteriorPoint();
                    newObstacle.updateBoundingBox();
                    obstacles.add(newObstacle);
                }
            }
            in.close();
        } catch (IOException e)
        {}

    }

    public synchronized void itemStateChanged(ItemEvent e)
       {
           Object src = e.getSource();
           boolean on = e.getStateChange() == ItemEvent.SELECTED;

        if (src == objEvaluationIntervalCombobox)
           {
               double newIncrement = Double.valueOf(objEvaluationIntervalCombobox.getSelectedItem().toString());
               sim.coverageMap = new double[(int) (boundary.largestY / newIncrement)][(int) (boundary.largestX / newIncrement)];
               sim.objEvalIncrement = newIncrement;

           }
           else if (src == this.scanIntervalCombobox)
           {
               double newIncrement = Double.valueOf(scanIntervalCombobox.getSelectedItem().toString());
                sim.scanMap = new double[(int) (boundary.largestY / newIncrement)][(int) (boundary.largestX / newIncrement)];
                sim.scanIncrement = newIncrement;
           }
    }

    public synchronized void actionPerformed(ActionEvent e)
      {
          Object src = e.getSource();

          if (src == scan)
             {
                scanMode=true;
                sim.scanWithOneAdditionalNode();

             }


          if (src == surfaceIntegralIncrementTextfield)
          {
              double newIncrement;
              try
              {
                  newIncrement = Double.valueOf(surfaceIntegralIncrementTextfield.getText());
              } catch (NumberFormatException ecp)
              {
                  newIncrement = -1;
              }

              if (newIncrement > 0 && newIncrement < 5)
              {
                  for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                  {
                      sim.robotList.get(i1).surface_integral_increment = newIncrement;
                  }
              }
              else
              {
                  surfaceIntegralIncrementTextfield.setText("(0-5)");
              }
          }

          if (src == nodeSpeedTextfield)
          {
              double newSpeed;
              try
              {
                  newSpeed = Double.valueOf(nodeSpeedTextfield.getText());
              } catch (NumberFormatException ecp)
              {
                  newSpeed = -1;
              }

              if (newSpeed > 0 && newSpeed < 99)
              {
                  for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                  {
                      sim.robotList.get(i1).robotMaxSpeed = newSpeed;
                  }
              }
              else
              {
                  nodeSpeedTextfield.setText("(0-99)");
              }
          }

          if (src == sensingDecayTextfield)
          {
              double newDecay;
              try
              {
                  newDecay = Double.valueOf(sensingDecayTextfield.getText());
              } catch (NumberFormatException ecp)
              {
                  newDecay = -1;
              }

              if (newDecay >= 0 && newDecay < 99)
              {
                  for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                  {
                      sim.robotList.get(i1).sensingDecayFactor = newDecay;
                  }
              }
              else
              {
                  sensingDecayTextfield.setText("[0-99)");
              }
          }

          if (src == sensingCutoffRangeTextfield)
          {
              double newRange;
              try
              {
                  newRange = Double.valueOf(sensingCutoffRangeTextfield.getText());
              } catch (NumberFormatException ecp)
              {
                  newRange = -1;
              }

              if (newRange >= -1 && newRange < 99)
              {
                  if(newRange<=0) //consider it infinite range
                  {
                      double longestRange = Math.sqrt( sim.boundary.largestX*sim.boundary.largestX+sim.boundary.largestY*sim.boundary.largestY);
                      this.sensingCutoffRangeTextfield.setText("inf");  //actually, it is not infinite.
                      this.sensingDecayTextfield.setText("0");
                     for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                      {
                          sim.robotList.get(i1).sensingCutoffRange = longestRange;
                          sim.robotList.get(i1).sensingDecayFactor = 0;
                      }

                  }
                  else
                  {
                      double adaptiveDecayFactor = -Math.log(0.01)/newRange;  //0.01 is not final
                      this.sensingDecayTextfield.setText(Double.toString(adaptiveDecayFactor).substring(0,5));
                      for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                      {
                          sim.robotList.get(i1).sensingCutoffRange = newRange;
                          sim.robotList.get(i1).sensingDecayFactor=adaptiveDecayFactor;
                      }
                  }
              }
              else
              {
                  sensingCutoffRangeTextfield.setText("[-1~99)");
              }
          }

          if (src == this.nominalGradientMag)
          {
              double newMagnitude;
              try
              {
                  newMagnitude = Double.valueOf(nominalGradientMag.getText());
              } catch (NumberFormatException ecp)
              {
                  newMagnitude = -1;
              }

              if (newMagnitude >= -1 && newMagnitude <= 1000000000)
              {
                  for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                  {
                      sim.robotList.get(i1).norminalGradientMagnitude = newMagnitude;
                  }
              }
              else
              {
                  nominalGradientMag.setText("[-1~1B]");
              }
          }

          if (src == this.updateObstaclesButton)
          {
              obstacles.clear();
              int obstacle_update_index = 0;
              String[] splitLines = obstaclesInputTextfield.getText().split("\n");
              for (int i1 = 0; i1 < splitLines.length; i1++)
              {
                  //    splitLines[i1] =  splitLines[i1].replaceAll(","," ");
                  splitLines[i1] = splitLines[i1].replaceAll("[\t,;a-zA-Z]", " ");
                  splitLines[i1] = splitLines[i1].replaceAll(" +", " ");
                  if (splitLines[i1].length() > 0)
                  {
                      if (splitLines[i1].charAt(0) == ' ') //eliminate preceding blanks
                      {
                          splitLines[i1] = splitLines[i1].replaceFirst(" ", "0");
                      }
                  }
                  try
                  {
                      String[] splitNumbers = splitLines[i1].split(" ");

                      if ((splitNumbers.length >= 6) && (splitNumbers.length % 2 == 0))
                      {
                          Obstacle newObstacle = new Obstacle();
                          for (int i = 0; i < splitNumbers.length / 2; i++)
                          {
                              newObstacle.vertices.add(new point2(Double.parseDouble(splitNumbers[i * 2]) + Math.random() * 0.00,
                                      Double.parseDouble(splitNumbers[i * 2 + 1]) + Math.random() * 0.00));
                              //add this random 0.01 is because human tends to create different objects using the same vertex. That will prevent that vertex
                              //becoming visible to nodes
                          }

                          obstacle_update_index++;

                          newObstacle.updateInteriorPoint();
                          newObstacle.updateBoundingBox();
                          obstacles.add(newObstacle);
                      }
                  } catch (NumberFormatException e1)
                  {

                  }
              }

              obstacle_number = obstacle_update_index;
              this.obstaclesForDrawing = new Polygon[obstacle_number];
              for (int i3 = 0; i3 < obstacle_number; i3++)
              {
                  obstaclesForDrawing[i3] = new Polygon();
                  for (int i2 = 0; i2 < obstacles.get(i3).vertices.size(); i2++)
                  {
                      obstaclesForDrawing[i3].addPoint((int) GraphPanel.zoomIn(obstacles.get(i3).vertices.get(i2).x),
                              (int) GraphPanel.zoomIn(obstacles.get(i3).vertices.get(i2).y));
                  }
              }
          }

          if (src == this.communicationWeightTextfield)
          {
              double newWeight;
              try
              {
                  newWeight = Double.valueOf(communicationWeightTextfield.getText());
              } catch (NumberFormatException ecp)
              {
                  newWeight = 0;
              }

              if (newWeight >= 0 && newWeight <= 100)
              {

                  for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                          {
                              sim.robotList.get(i1).communicationCostWeight = newWeight;
              }

              }
              else
              {
                  communicationWeightTextfield.setText("[0-100]");
              }
          }

          if (src == this.estimationErrorThresholdTextfield)
         {
             double newThreshold;
             try
             {
                 newThreshold = Double.valueOf(estimationErrorThresholdTextfield.getText());
             } catch (NumberFormatException ecp)
             {
                 newThreshold = 0;
             }

             if (newThreshold >= -1 && newThreshold <= 100)
             {

                 for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                         {
                             sim.robotList.get(i1).estimationErrorThreshold = newThreshold;
             }

             }
             else
             {
                 estimationErrorThresholdTextfield.setText("[-1,100]");
             }
         }

    }
}
