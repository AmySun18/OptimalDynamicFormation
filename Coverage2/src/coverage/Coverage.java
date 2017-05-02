package coverage;

import java.applet.*;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import java.net.*;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.BorderLayout;
import javax.swing.BorderFactory;
import javax.swing.table.*;

class Node
{
    double x = -1;
    double y = -1;

    boolean selected = false;
    // boolean fixed;
    int id = -1;
    String lbl;

    Node(){};
    
    Node(double x, double y, int _id, String _label)
    {
        x = GraphPanel.zoomIn(x);
        y = GraphPanel.zoomIn(y);
        id = _id;
        lbl = _label;
    }
}


public class Coverage extends Applet implements ActionListener, ItemListener, ChangeListener
{
    public Coverage()
    {
        try
        {
            jbInit();
        } catch (Exception ex)
        {
            ex.printStackTrace();
        }
    }

    Polygon obstaclesForDrawing[];
    int obstacle_number = 0;
    int world_size_x; //the dimension of the outer boundary
    int world_size_y;

    //demo control
    boolean start_demo = false; //TODO: if "true": start demo immediately after data load; else: click button to start demo

    //UI
    GraphPanel graphPanel;

    //JLabel dataDisplay = new JLabel();

    JPanel controlPanel = new JPanel();
    JButton pause = new JButton("Start");
    JButton reset_sim = new JButton("Reset");
    //JCheckBox realTimeCheckbox = new JCheckBox("Real Time Control");
    JCheckBox realTimeObjFuncValueCheckbox = new JCheckBox("Obj.Value:");
    JLabel realTimeObjFuncDisplay = new JLabel("-------------");

    //JLabel realTimeRobotNumberLabel = new JLabel("Nodes#");
    JSpinner realTimeRobotNumberSpinner;
    SpinnerNumberModel robotNumberSpinnerModel;
    
    JSpinner _activeDataSourceNumberSpinner;
    SpinnerNumberModel _activeDataSourceNumberSpinnerModel;

    JPanel rightPanel = new JPanel(new BorderLayout(10, 10));
    //The robot data table shown on the rightPanel
    JTable RobotTable;
    Object[][] table_data;
    //World geometry input
    JTextArea obstaclesInputTextfield;
    String obstaclesString = new String("");
    JButton updateObstaclesButton = new JButton("Update Obstacle Locations");

    //Algorithm parameter UI
    JTextField surfaceIntegralIncrementTextfield = new JTextField();
    JTextField nodeSpeedTextfield = new JTextField();
    JTextField sensingDecayTextfield = new JTextField();
    JTextField sensingCutoffRangeTextfield = new JTextField();
    JComboBox objEvaluationIntervalCombobox = new JComboBox();
    JTextField nominalGradientMag = new JTextField();

    //low detection boost
    JCheckBox lowDetectionBoostCheckbox = new JCheckBox("Low Detection Boost");
    public JTextField smallDetectionBoostTextfield = new JTextField();
    public JTextField UltraLowDetectionBoostTextfield = new JTextField();
    JLabel weightedObjFuncDisplay = new JLabel("--------------------");

    //Coverage control related
    boolean evaluateObjFunc = false; //control if we evaluate the objective function real-time, should be set to false normally
    Simulation sim;
    Obstacle boundary;
    ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>();
    double realTimeObjValue = -1;
    final int MAX_ROBOT_NUMBER = 25;

    //event density
    EventDensity density;
    boolean ShowEventDensity;
    JCheckBox eventDensityCheckbox = new JCheckBox("Density");
    
    //Aggregation stuff
    Aggregator aggregator;
    boolean InAggregationMode = false;
    boolean ShowAggregation = false;
    JButton doAggregationButton = new JButton("Est. Ini. Pos.");
    JCheckBox showAggregationCheckbox = new JCheckBox("Show Ini. Pos. Est.");
    JTextField attractionRangeTextfield = new JTextField();
    

    //Trajectory stuff
    JCheckBox showTrajectoryCheckbox = new JCheckBox("Show History");
    boolean showTrajectory = false;
    JButton clearTrajectoryButton = new JButton("Clear History");

    //Communication stuff
    JTextField communicationWeightTextfield = new JTextField();
    JLabel communication_cost_display = new JLabel("---------------");
    
//    JRadioButton randomOrder = new JRadioButton("Random");
//    JRadioButton breadthFirst = new JRadioButton("BreadthFirst");

    //Asynchronous, Position estimation stuff
    JTextField estimationErrorThresholdTextfield = new JTextField();
    JTextField KdeltaTextfield = new JTextField();

    //scan stuff, it is inadequate to put it here
    boolean scanMode = false;

    //event data collection parameters
    JTextField collectionDecayTextfield = new JTextField();
    JTextField dataCollectionWeightTextfield = new JTextField();
    
    //map building UI
    JTextField mapBuildingRangeTextfield = new JTextField();
    JCheckBox activeMapBuildingCheckbox = new JCheckBox("ActiveMapBuilding");
    
    //intrinsic communication delay
    JTextField commDelayTextfield = new JTextField();
    
    //communication range constraints
    JTextField comm_range_textfield = new JTextField();
    JButton moveLeaderButton = new JButton("Move Leader");

    
    //html parameters
    boolean is_obstacle_discovery_mode = false;
    int initial_node_num = 4;
    double max_node_speed = 8;
    boolean is_link_cost_mode = false;
    ArrayList<point2> robots_init_pos_list = new ArrayList<point2>();   
    boolean has_connectivity_constraint = false;
    double default_comm_range;
    boolean has_moving_data_source = false;
    double InitialSensorDecay = 0.057;
    private int _InitActiveDataSourceNum = 4;
    
    //relax the leaf nodes button
    JButton relaxLeafButton = new JButton("Relax Leaf");
    JTextField relax_strength_textfield = new JTextField();
    
    //FOV
    JTextField fov_range_textfield = new JTextField();
    
    //stochastic comparison
    boolean EnableStochasticSearch = false;
    JButton stochasticComparisonButton = new JButton("Random Search");

    public void init() //will be run only once unless the brower session is ended
    {
        //Dimension d = getSize();

        LoadEverything();

        //initialize some simulation related data structures
        density = new EventDensity(boundary.largestX,boundary.largestY,boundary.smallestX,boundary.smallestY);
        density.ActiveDataSourceNum = _InitActiveDataSourceNum;
        //density.initTrueEventDensity(world_size_x, world_size_y);
        sim = new Simulation(boundary, obstacles, density, initial_node_num, max_node_speed, robots_init_pos_list, default_comm_range, this, InitialSensorDecay); 
        aggregator = new Aggregator(boundary, obstacles, density);
        MakGUILayout();
        
        if ("true".equals(getParameter("boost_default_on")))
        {
        	lowDetectionBoostCheckbox.doClick();
        }
        
        if ("true".equals(getParameter("obj_eval_default_on")))
        {
        	realTimeObjFuncValueCheckbox.doClick();
        	
        }
     
        //You can put some unit test functions here
        //SmallTest();
    }

    private void LoadEverything() throws NumberFormatException
    {
        //read in string parameters (file names) from html
        String pos_datafile_name = getParameter("pos_data");
        if (pos_datafile_name == null)
        {
            pos_datafile_name = "robots_init_pos.txt";
        }
        String world_datafile_name = getParameter("world_data");
        if (world_datafile_name == null)
        {
            world_datafile_name = "cluttered_world.txt"; //the default mission space if not specified in the html
        }
        
      //read in numerical parameters from html
        String max_speed_str =  getParameter("max_speed");
        if (max_speed_str!=null)
        {
        	this.max_node_speed = Double.parseDouble(max_speed_str);
        }
        
        String comm_range_str =  getParameter("comm_range");
        if (comm_range_str!=null)
        {
        	this.default_comm_range= Double.parseDouble(comm_range_str);
        }
   
        String obstacle_detection_range_str =  getParameter("obstacle_detection_range");
        if (obstacle_detection_range_str!=null)
        {
        	Simulation.rangefinderRange= Double.parseDouble(obstacle_detection_range_str);
        }
        
        String initial_nodes_num_str = getParameter("init_nodes_num");
        if (initial_nodes_num_str != null)
        {
        	this.initial_node_num = Integer.parseInt(initial_nodes_num_str);
        }
        
        String initial_data_source_num_str = getParameter("init_data_source_num");
        if (initial_data_source_num_str != null)
        {
        	this._InitActiveDataSourceNum = Integer.parseInt(initial_data_source_num_str);
        }
        
        String sensorDecayStr = getParameter("sensor_decay");
        if (sensorDecayStr != null)
        {
        	this.InitialSensorDecay = Double.parseDouble(sensorDecayStr);
        }
        
        //get boolean html parameters       
        if (getParameter("discover_mode")!=null && getParameter("discover_mode").equals("true"))
        {
        	is_obstacle_discovery_mode = true;
        }
        
        if (getParameter("link_cost_mode")!=null && getParameter("link_cost_mode").equals("true"))
        {
        	is_link_cost_mode = true;
        }
        
        if (getParameter("has_connectivity_constraint")!=null && getParameter("has_connectivity_constraint").equals("true"))
        {
        	has_connectivity_constraint = true;
        }
        
        if (getParameter("has_moving_data_source")!=null && getParameter("has_moving_data_source").equals("true"))
        {
        	has_moving_data_source = true;
        }
        
        if (getParameter("enable_random_search")!=null && getParameter("enable_random_search").equals("true"))
        {
        	this.EnableStochasticSearch = true;
        }
        
        //load data file
        String line;
        URL url = null;
        URL url2 = null;

        try
        {
            url = new URL(getCodeBase(), pos_datafile_name);
            url2 = new URL(getCodeBase(), world_datafile_name);
        } catch (MalformedURLException e)
        {
            System.out.println("Malformed URL ");
            stop();
        }

       
        	BufferedReader dis;
        	String[] splitString;
        	InputStream in;
            //load robots initial positions
        	 try
             {
               in = url.openStream();

               dis = new BufferedReader(new InputStreamReader(in));
               Integer.parseInt(dis.readLine()); //first line of the file indicates how many lines are left. Not useful for now. So just read and skip it          
               while ((line = dis.readLine()) != null)
               {
               	line = line.replaceAll("[\t,;a-zA-Z]", " ");
                   line = line.replaceAll(" +", " ");
                   splitString = line.split(" ");
                   if (splitString.length == 2) //x and y
                   {
                   	this.robots_init_pos_list.add(new point2(Double.parseDouble(splitString[0]),Double.parseDouble(splitString[1])));
                   }
               }
               in.close();
               
             }
        	 catch (IOException e)
             {
             	System.out.println(e.getMessage());      	
             }

        	 
            //load world geometry for display and simulation
            try
            {
            	
            in = url2.openStream();
             
            dis = new BufferedReader(new InputStreamReader(in));
            this.world_size_x = Integer.parseInt(dis.readLine());
            this.world_size_y = Integer.parseInt(dis.readLine());

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
                    
                    //do not need this if obstacles are known beforehand
                    if(is_obstacle_discovery_mode)
                    {
                    	newObstacle.fillinVertices();
                    }
                    else
                    {
                    	newObstacle.originalVertices = (ArrayList<point2>) newObstacle.vertices.clone();
                    }
                    
                    obstacles.add(newObstacle);
                }
            }
            in.close();
        
            }
            catch (IOException e)
            {
            	e.printStackTrace();            	
            	this.world_size_x = 60;
                this.world_size_y = 50;
                this.obstacle_number = 0;
                obstaclesForDrawing = new Polygon[this.obstacle_number];    	
            }
            
            boundary = new Obstacle();  //here we make sure that boundary is a rectangle
            boundary.vertices.add(new point2(0, 0));
            boundary.vertices.add(new point2(this.world_size_x, 0));
            boundary.vertices.add(new point2(this.world_size_x, this.world_size_y));
            boundary.vertices.add(new point2(0, this.world_size_y));
            boundary.originalVertices = (ArrayList<point2>) boundary.vertices.clone();
            boundary.updateBoundingBox();
                    
    }

    private void MakGUILayout()
    {
        setLayout(new BorderLayout());

        if(is_obstacle_discovery_mode)
        {
        	graphPanel = new MapBuildingGraphPanel(this);
        }
        else
        {
        	graphPanel = new GraphPanel(this); //change this to MapBuildingGraphPanel when obstacles are unknown
        }
        add("Center", graphPanel);

        //North Panel
        JPanel northPanel = new JPanel();
        //  northPanel.setLayout(new BoxLayout(northPanel,BoxLayout.LINE_AXIS));
        add("North", northPanel);

        pause.setMargin(new Insets(5,2,5,2)); //top, left, bottom, right
        pause.setPreferredSize(new Dimension(60,25));
        northPanel.add(pause);
        pause.addActionListener(this);
        reset_sim.setMargin(new Insets(5,2,5,2)); //top, left, bottom, right
        reset_sim.setPreferredSize(new Dimension(50,25));
        northPanel.add( reset_sim);
        reset_sim.addActionListener(this);
        
        if(this.has_connectivity_constraint)
        {
	        northPanel.add(this.relaxLeafButton);
	        relaxLeafButton.addActionListener(this);
	        northPanel.add(new JLabel("strength"));
	        northPanel.add(relax_strength_textfield);
	        relax_strength_textfield.addActionListener(this);
	        relax_strength_textfield.setText(" 1.5 ");
        }
        
        if(this.EnableStochasticSearch)
        {
        	northPanel.add(this.stochasticComparisonButton);
        	stochasticComparisonButton.addActionListener(this);	
        }
        
        // robot number control
        robotNumberSpinnerModel = new SpinnerNumberModel(this.sim.realTimeRobotNumber, 1, this.MAX_ROBOT_NUMBER, 1);
        this.realTimeRobotNumberSpinner = new JSpinner(this.robotNumberSpinnerModel);
        northPanel.add(new JLabel("Nodes#"));
        northPanel.add(this.realTimeRobotNumberSpinner);
        robotNumberSpinnerModel.addChangeListener(this);
        //this.realTimeRobotNumberSpinner.setEnabled(true);
       
        //active data source number control
        if(has_moving_data_source)
        {
	        _activeDataSourceNumberSpinnerModel = new SpinnerNumberModel(density.ActiveDataSourceNum, 0, density.MaxDataSourceNum, 1);
	        this._activeDataSourceNumberSpinner = new JSpinner(this._activeDataSourceNumberSpinnerModel);
	        northPanel.add(new JLabel("DataSource#"));
	        northPanel.add(this._activeDataSourceNumberSpinner);
	        _activeDataSourceNumberSpinnerModel.addChangeListener(this);
	        
	        northPanel.add(eventDensityCheckbox);
	        eventDensityCheckbox.addItemListener(this);
        }
        
        insertVerticalSeparator(northPanel);

        realTimeObjFuncValueCheckbox.setBackground(Color.PINK);
        northPanel.add(realTimeObjFuncValueCheckbox);
        realTimeObjFuncValueCheckbox.addItemListener(this);
        //realTimeObjFuncValueCheckbox.setEnabled(true);
        realTimeObjFuncDisplay.setBackground(Color.PINK);
        realTimeObjFuncDisplay.setOpaque(true);
        realTimeObjFuncDisplay.setMaximumSize(new Dimension(20, 20));
        northPanel.add(this.realTimeObjFuncDisplay);
        //realTimeObjFuncDisplay.setEnabled(true);

        northPanel.add(new JLabel("Res."));
        northPanel.add(objEvaluationIntervalCombobox);
        objEvaluationIntervalCombobox.addItem(0.25);
        objEvaluationIntervalCombobox.addItem(0.5);
        objEvaluationIntervalCombobox.addItem(1);
        objEvaluationIntervalCombobox.setSelectedIndex(2);
        objEvaluationIntervalCombobox.addItemListener(this);
       // objEvaluationIntervalCombobox.setEnabled(true);
        
        insertVerticalSeparator(northPanel);

        //northPanel.add(new JLabel("Low Detection Boost"));
        northPanel.add(this.lowDetectionBoostCheckbox);
        lowDetectionBoostCheckbox.addItemListener(this);
        northPanel.add(smallDetectionBoostTextfield);
        //  smallDetectionBoostTextfield.setText(Double.toString(sim.robotList.get(0).smallDetectionBoost));
        smallDetectionBoostTextfield.setText("1000");
        smallDetectionBoostTextfield.setPreferredSize(new Dimension(60, 20));
        smallDetectionBoostTextfield.addActionListener(this);
        smallDetectionBoostTextfield.setEnabled(false);
        

        northPanel.add(UltraLowDetectionBoostTextfield);
        //  UltraLowDetectionBoostTextfield.setText(Double.toString(sim.robotList.get(0).ultraLowDetectionBoost));
        UltraLowDetectionBoostTextfield.setText("1000000");
        UltraLowDetectionBoostTextfield.setPreferredSize(new Dimension(60, 20));
        UltraLowDetectionBoostTextfield.addActionListener(this);
        UltraLowDetectionBoostTextfield.setEnabled(false);

        northPanel.add(weightedObjFuncDisplay);
        //    weightedObjFuncDisplay.setEnabled(false);

        insertVerticalSeparator(northPanel);

        northPanel.add(this.showTrajectoryCheckbox);
        showTrajectoryCheckbox.addItemListener(this);
        northPanel.add(this.clearTrajectoryButton);
        clearTrajectoryButton.addActionListener(this);

//Aggregation stuff, hide for now
        //northPanel.add(doAggregationButton);
        //doAggregationButton.addActionListener(this);
        //northPanel.add(showAggregationCheckbox);
        // showAggregationCheckbox.addItemListener(this);

        //  northPanel.add(new JLabel("Attraction Range"));
        //  northPanel.add(this.attractionRangeTextfield);
        //  attractionRangeTextfield.setText(Double.toString(this.aggregator.attractionRange));
        //   attractionRangeTextfield.addActionListener(this);
// end aggregation stuff

        //right panel
        rightPanel.setPreferredSize(new Dimension(225, 400));
        add("East", rightPanel);

        rightPanel.setLayout(new FlowLayout());

     
        //setup robot info display
        String[] columnNames =
                {"ID", "X", "Y", "Hi(s)" , "Heading", "dH1x", "dH1y", "dH2x", "dH2y"};
        table_data = new Object[MAX_ROBOT_NUMBER][9];
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
        RobotTable.setPreferredScrollableViewportSize(new Dimension(200, 230)); //control the size of robot table here
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
        
        //**Shink idea related 
//        ButtonGroup group = new ButtonGroup();
//        group.add(randomOrder);
//        group.add(breadthFirst);
//        rightPanel.add(randomOrder);
//        randomOrder.doClick();
//        randomOrder.addItemListener(this);
//        rightPanel.add(breadthFirst);
//        breadthFirst.addItemListener(this);
        
        rightPanel.add(moveLeaderButton);
        moveLeaderButton.setPreferredSize(new Dimension(200,20));
        moveLeaderButton.addActionListener(this);
        


        
        rightPanel.add(this.updateObstaclesButton);

        obstaclesInputTextfield = new JTextArea();
        obstaclesInputTextfield.setBorder(BorderFactory.createEtchedBorder());
        obstaclesInputTextfield.setText(obstaclesString);
        obstaclesInputTextfield.setTabSize(2);
        JScrollPane scrollPane1 = new JScrollPane(obstaclesInputTextfield);
        scrollPane1.setPreferredSize(new Dimension(200, 310)); //control the size of obstacle list here
        rightPanel.add("South", scrollPane1);

        updateObstaclesButton.setPreferredSize(new Dimension(200, 40));
        updateObstaclesButton.setMaximumSize(new Dimension(200, 80));
       // rightPanel.add("Center", this.updateObstaclesButton);
       
        updateObstaclesButton.addActionListener(this);

        //Control Parameters
        add("South", controlPanel);

        if(this.is_obstacle_discovery_mode)
        {
	        controlPanel.add(new JLabel("MB Range"));
	        controlPanel.add(mapBuildingRangeTextfield);
	        mapBuildingRangeTextfield.setText(Double.toString(Simulation.rangefinderRange)+" ");
	        mapBuildingRangeTextfield.addActionListener(this);
	    //    mapBuildingRangeTextfield.setEnabled(true);
	        
	        controlPanel.add(activeMapBuildingCheckbox);
	        activeMapBuildingCheckbox.setBackground(Color.green);
	        activeMapBuildingCheckbox.addItemListener(this);
        }
        
        controlPanel.add(new JLabel("FOV"));
        controlPanel.add(fov_range_textfield);
        fov_range_textfield.setText(Double.toString(sim.robotList.get(0).FOV_degree).substring(0, 5)+" ");
        fov_range_textfield.addActionListener(this);
        //fov_range_textfield.setEnabled(true);
        
        controlPanel.add(new JLabel("Delay"));
        controlPanel.add(commDelayTextfield);
        commDelayTextfield.setText(Double.toString(sim.robotList.get(0).comm_delay)+" ");
        commDelayTextfield.addActionListener(this);
        //commDelayTextfield.setEnabled(true);
        
        if(this.has_connectivity_constraint)
        {
	        controlPanel.add(new JLabel("CommRange"));
	        controlPanel.add(comm_range_textfield);
	        comm_range_textfield.setText(Double.toString(sim.communication_range)+" ");
	        comm_range_textfield.addActionListener(this);
	       // comm_range_textfield.setEnabled(true);
        }
        
        
        controlPanel.add(new JLabel("MaxSpeed"));
        controlPanel.add(nodeSpeedTextfield);
        nodeSpeedTextfield.setText(Double.toString(sim.robotList.get(0).robotMaxSpeed)+" ");
        nodeSpeedTextfield.addActionListener(this);
       // nodeSpeedTextfield.setEnabled(true);

        controlPanel.add(new JLabel("SensingDecay"));
        controlPanel.add(sensingDecayTextfield);
        sensingDecayTextfield.setText(Double.toString(sim.robotList.get(0).sensingDecayFactor));
        sensingDecayTextfield.addActionListener(this);
       // sensingDecayTextfield.setEnabled(true);
        //sensingDecayTextfield.setEditable(false); //if sensing decay related to "Sensing Range", then it should not be editable

        controlPanel.add(new JLabel("SensingRange"));
        controlPanel.add(sensingCutoffRangeTextfield);
        sensingCutoffRangeTextfield.setText(Double.toString(sim.robotList.get(0).sensingCutoffRange));
        sensingCutoffRangeTextfield.addActionListener(this);
       // sensingCutoffRangeTextfield.setEnabled(true);

        controlPanel.add(new JLabel("MaxNorm"));
        controlPanel.add(this.nominalGradientMag);
        nominalGradientMag.setText(Double.toString(sim.robotList.get(0).norminalGradientMagnitude));
        nominalGradientMag.addActionListener(this);
       // nominalGradientMag.setEnabled(true);


        controlPanel.add(new JLabel("Kdelta"));
        controlPanel.add(this.KdeltaTextfield);
        KdeltaTextfield.setText(Double.toString(sim.robotList.get(0).K4) + "      ");
        KdeltaTextfield.addActionListener(this);


        controlPanel.add(new JLabel("Est.Threshold"));
         controlPanel.add(this.estimationErrorThresholdTextfield);
         estimationErrorThresholdTextfield.setText(Double.toString(sim.robotList.get(0).estimationErrorThreshold) + "      ");
        estimationErrorThresholdTextfield.addActionListener(this);

        if(this.is_link_cost_mode){
        controlPanel.add(new JLabel("Comm.Weight"));
        controlPanel.add(this.communicationWeightTextfield);
        communicationWeightTextfield.setText(Double.toString(sim.robotList.get(0).communicationCostWeight) + "      ");
        communicationWeightTextfield.addActionListener(this);
        
        controlPanel.add(new JLabel("LinkCost:"));
        controlPanel.add(communication_cost_display);
              
        }


        controlPanel.add(new JLabel("CollectDecay"));
        controlPanel.add(this.collectionDecayTextfield);
        collectionDecayTextfield.setText(Double.toString(sim.robotList.get(0).dataCollectionDecay) + "      ");
        collectionDecayTextfield.addActionListener(this);

        controlPanel.add(new JLabel("CollectWeight"));
        controlPanel.add(this.dataCollectionWeightTextfield);
        dataCollectionWeightTextfield.setText(Double.toString(sim.robotList.get(0).dataCollectionWeight) + "      ");
        dataCollectionWeightTextfield.addActionListener(this);

        controlPanel.add(new JLabel("Int. Res.")); //The integral resolution
        controlPanel.add(this.surfaceIntegralIncrementTextfield);
        //  surfaceIntegralIncrementTextfield.setPreferredSize(new Dimension(40,21));
        surfaceIntegralIncrementTextfield.setText(Double.toString(sim.robotList.get(0).surface_integral_increment));
        surfaceIntegralIncrementTextfield.addActionListener(this);
        //surfaceIntegralIncrementTextfield.setEnabled(true);

        //  insertVerticalSeparator(controlPanel);

    }

    public void insertVerticalSeparator(JPanel panel)
    {
        panel.add(Box.createHorizontalStrut(10));
        JSeparator separate0 = new JSeparator(SwingConstants.VERTICAL);
        panel.add(separate0);
        separate0.setPreferredSize(new Dimension(1, 24));
        panel.add(Box.createHorizontalStrut(10));
    }

    public void updateTable()
    {
            for (int i5 = 0; i5 < this.MAX_ROBOT_NUMBER; i5++)
            {
            	
                {
                    RobotTable.setValueAt(sim.robotList.get(i5).position.x, i5, 1);
                    RobotTable.setValueAt(sim.robotList.get(i5).position.y, i5, 2);
                    RobotTable.setValueAt(sim.robotList.get(i5).his_informationValue, i5, 3);
                }
            }      
    }


    public void destroy()
    {
        remove(graphPanel);
        remove(controlPanel);
        remove(rightPanel);
    }

    public void start()
    {
        graphPanel.start();

    }

    public void stop()
    {
        graphPanel.stop();

    }

    public synchronized void actionPerformed(ActionEvent e)
    {
        Object src = e.getSource();

        if (src == pause)
        {
            this.start_demo = !this.start_demo;
            if (start_demo)
            {
                pause.setText("Pause");
            }
            else
            {
                pause.setText("Continue");
            }
        }

        if (src == reset_sim)
        {
        	if(this.start_demo)
        	{
        		this.sim.to_reset = true;
        	}
        	else
        	{
        		this.sim.reset_robots();
        		this.sim.ResetObstacles();
        		this.graphPanel.read_nodes_pos();
        		this.updateTable();
        	}
        }

        if(src == moveLeaderButton)
        {
        	if(sim.robotList.get(0).position.x < sim.boundary.largestX && sim.robotList.get(0).position.y < sim.boundary.largestY)
        	{
//        		for(int i=0; i<sim.realTimeRobotNumber;i++)
        		{
        			sim.robotList.get(0).position.x = sim.robotList.get(0).position.x + 1;
        			sim.Projection=true;
//        	        sim.robotList.get(i).position.y = sim.robotList.get(i).position.y;
        		}
        	}
        	
        }
        
//        if(src == projectionButton)
//        {
//        	sim.Projection=true;        	
//        }

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
        
        if (src == mapBuildingRangeTextfield)
        {
            double newRange;
            try
            {
            	newRange = Double.valueOf(mapBuildingRangeTextfield.getText());
            } catch (NumberFormatException ecp)
            {
            	newRange = -1;
            }

            if (newRange > 0 && newRange < 99)
            {
                
            	Simulation.rangefinderRange = newRange;          
            }
            else
            {
            	mapBuildingRangeTextfield.setText("(0-99)");
            }
        } 
        
        if (src == this.fov_range_textfield)
        {
            double newRange;
            try
            {
            	newRange = Double.valueOf(fov_range_textfield.getText());
            } 
            catch (NumberFormatException ecp)
            {
            	fov_range_textfield.setText(">0&<=2PI");
            	return;
            }

            if (newRange < 0)
            {
            	fov_range_textfield.setText("0");
            	newRange = 0;
            }
            else if (newRange >Math.PI*2)
            {
            	fov_range_textfield.setText("6.283");
            		newRange = Math.PI*2;
            }
            
            for (int i1 = 0; i1 < sim.robotList.size(); i1++)
            {
                sim.robotList.get(i1).FOV_degree = newRange;
            }  
            
            return;
        } 
        
        if (src == comm_range_textfield)
        {
            double new_range;
            try
            {
            	new_range = Double.valueOf(comm_range_textfield.getText());
            } catch (NumberFormatException ecp)
            {
            	new_range = -1;
            }

            if (new_range > 0 && new_range < 99)
            {
            	
                sim.communication_range = new_range;   
            }
            else
            {
            	comm_range_textfield.setText("(0-99)");
            }
        }
        
        if (src == commDelayTextfield)
        {
            double newDelay;
            try
            {
            	newDelay = Double.valueOf(commDelayTextfield.getText());
            } catch (NumberFormatException ecp)
            {
            	newDelay = -1;
            }

            if (newDelay >= 0 && newDelay <= 999)
            {
            	for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                {
                    sim.robotList.get(i1).comm_delay = (int)newDelay;
                }       
            }
            else
            {
            	commDelayTextfield.setText("[0-999)");
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
                if (newRange <= 0) //consider it infinite range
                {
                    double longestRange = Math.sqrt(sim.boundary.largestX * sim.boundary.largestX +
                            sim.boundary.largestY * sim.boundary.largestY);
                    this.sensingCutoffRangeTextfield.setText("inf"); //actually, it is not infinite.
                    //this.sensingDecayTextfield.setText("0");
                    for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                    {
                        sim.robotList.get(i1).sensingCutoffRange = longestRange;
                        sim.robotList.get(i1).sensingDecayFactor = 0;
                    }

                }
                else
                {
                    //double adaptiveDecayFactor = -Math.log(0.01) / newRange; //0.01 is not final                 
                    for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                    {
                        sim.robotList.get(i1).sensingCutoffRange = newRange;
                       // sim.robotList.get(i1).set_decay_from_range();
                       // sim.robotList.get(i1).sensingDecayFactor = adaptiveDecayFactor;
                    }
                    //this.sensingDecayTextfield.setText(Double.toString(sim.robotList.get(0).sensingDecayFactor).substring(0, 5));
                }
            }
            else
            {
                sensingCutoffRangeTextfield.setText("[-1~99)");
            }
        }

        if (src == smallDetectionBoostTextfield)
        {
            double newBoost;
            try
            {
                newBoost = Double.valueOf(smallDetectionBoostTextfield.getText());
            } catch (NumberFormatException ecp)
            {
                newBoost = -1;
            }

            if (newBoost >= 1 && newBoost <= 1000000000)
            {
                for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                {
                    sim.robotList.get(i1).smallDetectionBoost = newBoost;
                }
            }
            else
            {
                smallDetectionBoostTextfield.setText("[1-1B]");
            }
        }

        if (src == this.UltraLowDetectionBoostTextfield)
        {
            double newBoost;
            try
            {
                newBoost = Double.valueOf(UltraLowDetectionBoostTextfield.getText());
            } catch (NumberFormatException ecp)
            {
                newBoost = -1;
            }

            if (newBoost >= 1 && newBoost <= 1000000000)
            {
                for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                {
                    sim.robotList.get(i1).ultraLowDetectionBoost = newBoost;
                }
            }
            else
            {
                UltraLowDetectionBoostTextfield.setText("[1-1B]");
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
        
        if (src == this.relaxLeafButton)
        {
        	//this.sim.relax_leaf_nodes();
        	sim.relax_leaf_nodes_alt();
        }
        
//        if (src == this.stochasticComparisonButton)
//        {
//        	StochasticComparisonTask task = new StochasticComparisonTask(this);
//        	task.execute();
//        	this.graphPanel.stop();
//        	if(this.start_demo)
//        	{
//	        	this.start_demo = false;
//	        	StochasticComparison.ImproveRobotStates(sim);
//	        	//StochasticComparison.ImproveRobotAngles(sim);
//	        	this.start_demo = true;
//        	}
//        	else
//        	{
//        		StochasticComparison.ImproveRobotStates(sim);
//        		this.graphPanel.read_nodes_pos();
//        		this.updateTable();
//        	}
//        	this.graphPanel.start();
//        }
        
        
        if (src == this.updateObstaclesButton)
        {
        	obstacle_number = 0;
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
                        
                        if(this.is_obstacle_discovery_mode)
                        {
                        	newObstacle.fillinVertices();
                        }
                        else
                        {
                        	newObstacle.originalVertices = (ArrayList<point2>) newObstacle.vertices.clone();
                        }
                        
                        obstacles.add(newObstacle);
                    }
                } catch (NumberFormatException e1)
                {
                	e1.printStackTrace();     
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

        if (src == this.doAggregationButton)
        {
            //if already doing aggregation, do nothing here
            if (!InAggregationMode)
            {
                //first pause the simulation
                this.start_demo = false;
                pause.setText("Continue");

                aggregator.CreateElements();
                this.InAggregationMode = true;

            }
        }

        if (src == this.attractionRangeTextfield)
        {
            double newRange;
            try
            {
                newRange = Double.valueOf(attractionRangeTextfield.getText());
            } catch (NumberFormatException ecp)
            {
                newRange = -1;
            }

            if (newRange >= 1 && newRange <= 100)
            {

                this.aggregator.attractionRange = newRange;

            }
            else
            {
                attractionRangeTextfield.setText("[1-100]");
            }

        }

        if (src == this.clearTrajectoryButton)
        {
            for (int i1 = 0; i1 < sim.robotList.size(); i1++)
            {
                sim.robotList.get(i1).trajectoryHistoryIndex = 0;
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

            if (newWeight >= 0 && newWeight <= 100000)
            {

                for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                {
                    sim.robotList.get(i1).communicationCostWeight = newWeight;
                }

            }
            else
            {
                communicationWeightTextfield.setText("[0-100000]");
            }
        }

         if (src == this.KdeltaTextfield)
       {
           double newKdelta;
           try
           {
               newKdelta = Double.valueOf(KdeltaTextfield.getText());
           } catch (NumberFormatException ecp)
           {
               newKdelta = 0;
           }

           if (newKdelta >= 0 && newKdelta <= 100)
           {

               for (int i1 = 0; i1 < sim.robotList.size(); i1++)
               {
                   sim.robotList.get(i1).K4 = newKdelta;
                   sim.robotList.get(i1).isFixedError = false;
               }

           }
           else
           {
               KdeltaTextfield.setText("[0,100]");
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
                     sim.robotList.get(i1).isFixedError = true;
                }

            }
            else
            {
                estimationErrorThresholdTextfield.setText("[-1,100]");
            }
        }

        if (src == this.collectionDecayTextfield)
        {
            double newDecay;
            try
            {
                newDecay = Double.valueOf(collectionDecayTextfield.getText());
            } catch (NumberFormatException ecp)
            {
                newDecay = 0;
            }

            if (newDecay >= 0 && newDecay <= 100)
            {

                for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                {
                    sim.robotList.get(i1).dataCollectionDecay = newDecay;
                }

            }
            else
            {
                collectionDecayTextfield.setText("[0,100]");
            }
        }

        if (src == this.dataCollectionWeightTextfield)
        {
            double newWeight;
            try
            {
                newWeight = Double.valueOf(dataCollectionWeightTextfield.getText());
            } catch (NumberFormatException ecp)
            {
                newWeight = 0;
            }

            if (newWeight >= 0 && newWeight <= 100000000)
            {

                for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                {
                    sim.robotList.get(i1).dataCollectionWeight = newWeight;
                }

            }
            else
            {
                dataCollectionWeightTextfield.setText("[0,1E8]");
            }

        }
        
        if (src == this.relax_strength_textfield)
        {
            double new_strength;
            try
            {
            	new_strength = Double.valueOf(relax_strength_textfield.getText());
            } catch (NumberFormatException ecp)
            {
            	new_strength = 1.5;
            }

            if (new_strength >= 1 && new_strength <= 1000)
            {
            	sim.leaf_relax_strength = new_strength;
            }
            else
            {
            	relax_strength_textfield.setText("[1,100]");
            }

        }
        
    }

    public synchronized void itemStateChanged(ItemEvent e)
    {
        Object src = e.getSource();
        boolean on = e.getStateChange() == ItemEvent.SELECTED;

       if (src == realTimeObjFuncValueCheckbox)
        {
            evaluateObjFunc = on;
            if (evaluateObjFunc)
            {
                 sim.UpdateObjInfo();         
            }
        }
//       else if (src == this.randomOrder)
//       {
//    	   if(on == true)
//    	   {
//    		  sim.RandomOrder = true;
//    	   }
//    	   else
//    	   {
//    		   sim.RandomOrder = false;
//    	   }
//    	   
//       }
//       else if(src == this.breadthFirst)
//       {
//    	   if(on == true)
//    	   {
//    		   sim.BreadthFirst = true;
//    	   }
//    	   else
//    	   {
//    		   sim.BreadthFirst = false;
//    	   }
//       }
       
        else if (src == this.lowDetectionBoostCheckbox)
        {
            smallDetectionBoostTextfield.setEnabled(on);
            this.UltraLowDetectionBoostTextfield.setEnabled(on);
            //  weightedObjFuncDisplay.setEnabled(on);
            // weightedObjFuncDisplay.setVisible(on);
            if (on == true)
            {
                //  smallDetectionBoostTextfield.setText("1000");
                //  UltraLowDetectionBoostTextfield.setText("1000000");
                for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                {
                    sim.robotList.get(i1).smallDetectionBoost = 1000;
                    sim.robotList.get(i1).ultraLowDetectionBoost = 1000000;
                }

            }
            else
            {
                //   smallDetectionBoostTextfield.setText("1");
                //   UltraLowDetectionBoostTextfield.setText("1");
                for (int i1 = 0; i1 < sim.robotList.size(); i1++)
                {
                    sim.robotList.get(i1).smallDetectionBoost = 1;
                    sim.robotList.get(i1).ultraLowDetectionBoost = 1;
                }

            }
        }
        else if (src == objEvaluationIntervalCombobox)
        {
            double newIncrement = Double.valueOf(objEvaluationIntervalCombobox.getSelectedItem().toString());
            sim.coverageMap = new double[(int) (boundary.largestY / newIncrement)][(int) (boundary.largestX / newIncrement)];
            sim.objEvalIncrement = newIncrement;
            
            if (evaluateObjFunc)
            {
                 sim.UpdateObjInfo();         
            }

        }
        else if (src == this.showTrajectoryCheckbox)
        {
            this.showTrajectory = on;
            for (int i1 = 0; i1 < sim.robotList.size(); i1++)
            {
                sim.robotList.get(i1).recordTrajectory = on;
            }

        }
        else if (src == activeMapBuildingCheckbox)
        {
        	 for (int i1 = 0; i1 < sim.robotList.size(); i1++)
             {
                 sim.robotList.get(i1).inActiveMapBuildingMode = on;
             }
        }
        else if (src == eventDensityCheckbox)
        {
        	this.ShowEventDensity = on;
        }

    }

    public synchronized void stateChanged(ChangeEvent evt)
    {
        SpinnerNumberModel source = (SpinnerNumberModel) evt.getSource();

        if (source == this.robotNumberSpinnerModel)
        {
            sim.realTimeRobotNumber = source.getNumber().intValue();

            sim.updateEventCollectionPercentage();
            if(this.is_link_cost_mode || this.has_connectivity_constraint)
            {
            	sim.pathGenerator.buildRoutingTree();
            }
            if (evaluateObjFunc)
            {
                sim.UpdateObjInfo(); 
            }

        }
        
        if (source == _activeDataSourceNumberSpinnerModel)
        {
            density.ActiveDataSourceNum = source.getNumber().intValue();

            if (evaluateObjFunc)
            {
                sim.UpdateObjInfo(); 
            }

        }
    }

    public String getAppletInfo()
    {
        return "Title: Coverage Demo \nAuthor: Minyi Zhong";
    }

    public String[][] getParameterInfo()
    {
        String[][] info =
                {
                {"fixme", "fixme", "fixme"}
                // {"edges", "delimited string", "A comma-delimited list of all the edges.  It takes the form of 'C-N1,C-N2,C-N3,C-NX,N1-N2/M12,N2-N3/M23,N3-NX/M3X,...' where C is the name of center node (see 'center' parameter) and NX is a node attached to the center node.  For the edges connecting nodes to eachother (and not to the center node) you may (optionally) specify a length MXY separated from the edge name by a forward slash."},
                // {"center", "string", "The name of the center node."}
        };
        return info;
    }

    private void jbInit() throws Exception
    {

    }

    private void SmallTest()
    {
        //TODO: function test, should go away in final release
        point2 p1 = new point2(3.5, 4);
        point2 p3 = new point2(7, 1);
        point2 p2 = new point2(2, 1);
        point2 p4 = new point2(7, 7);
        point2 testPoint = new point2(((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p3.x * p4.y - p3.y * p4.x) * (p1.x - p2.x)) /
                                      ((p1.x - p2.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p2.y)),
                                      ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p3.x * p4.y - p3.y * p4.x) * (p1.y - p2.y)) /
                                      ((p1.x - p2.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p2.y)));
        System.out.println(testPoint.x);
        System.out.println(testPoint.y);
    }
}

