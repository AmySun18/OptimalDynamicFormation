package coverage;

import java.util.ArrayList;

public class MapBuildingRobot extends Robot {

	public MapBuildingRobot() {
	}

	public MapBuildingRobot(int _id, point2 p, double h, double leftS,
			double rightS, double w, double robotMaxS,
			double lastStatesUpdateT, double controlP,
			double lastControlT, Obstacle bndr, ArrayList<Obstacle> gswl,
			EventDensity dnst, Simulation s, double sensing_decay) {
		super(_id, p, h, leftS, rightS, w, robotMaxS, lastStatesUpdateT,
				 controlP, lastControlT, bndr, gswl, dnst, s, sensing_decay);
	}
	
	synchronized void SpeedControlAlgo(double currentTime)
	{
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
 
        for (double horizontalSamplePoint = 0.01; horizontalSamplePoint <= boundary.largestX; horizontalSamplePoint += delta)
        {        
            for (double verticalSamplePoint = 0.01; verticalSamplePoint <= boundary.largestY; verticalSamplePoint += delta)
            {
                samplePoint.x = horizontalSamplePoint;
                samplePoint.y = verticalSamplePoint;
                dist = point2.Dist(samplePoint, position);
                if (dist < this.sensingCutoffRange && dist > 0) //SYNC
                {
                    //boundary wall is concrete, it doesn't allow any signal to go through. So a point is meaningless to observer if it is blocked by boundary wall.
                    if (boundary.LineOfSight(position, samplePoint))
                    {
                        alpha = 1;

                        for (int i1 = 0; i1 < obstacles.size(); i1++)
                        {
                            if (!obstacles.get(i1).MapBuildingLineOfSight (position, samplePoint))
                            {
                                alpha = EventDensity.WALL_DECAY_FACTOR;
                                break;
                            }
                        }
                        //Check Wei's paper for this equation.
                        if (alpha > 0) //performance concern, often EventDensity.WALL_DECAY_FACTOR is 0;
                        {
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
            }
        }

        //long midTime = myTimer.currentTimeMillis();  //for performance tuning

        //derivative due to corner points
        dFdx2 = 0;
        dFdy2 = 0;

        ArrayList<point2> activeReflexVertices = new ArrayList<point2>();

        //Assuming the boundary is convex

        for (int i2 = 0; i2 < obstacles.size(); i2++)
        {
            obstacles.get(i2).GetMapBuildingActiveReflexVertices(position, activeReflexVertices); //must clear activeReflexVertices before calling this again

            for (int i3 = 0; i3 < activeReflexVertices.size(); i3++)
            {
                double D = point2.Dist(activeReflexVertices.get(i3), position); //SYNC
                if (D < this.sensingCutoffRange) //SYNC
                {
                    boolean ARVBlocked = false; //test if an ARV is blocked by other FG
                    
                    if (!(boundary.LineOfSight(position, activeReflexVertices.get(i3)))) //SYNC
                    {
                        ARVBlocked = true;
                    }
                    else
                    {
                        for (int i4 = 0; i4 < obstacles.size(); i4++)
                        {
                            if (i2 != i4)
                            {
                                if (!(obstacles.get(i4).MapBuildingLineOfSight (position, activeReflexVertices.get(i3))))
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
                                obstacles.get(i5).GetMapBuildingImpactPoint(activeReflexVertices.get(i3),
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
                        double delta1 = 0.2; //was 0.05 //k: expose this parameter in GUI
                        double integrationUpperBound = d; //SYNC
                        if ((D + d) > this.sensingCutoffRange)
                        {
                            integrationUpperBound = this.sensingCutoffRange - D;
                        }

                        point2 integralPoint = new point2();
                        for (double r = delta1 / 2; r <= integrationUpperBound; r += delta1) //SYNC
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

                            /*if(Phat<0.1)
                             {
                                 integralSum += ultraLowDetectionBoost* delta1 * r * density.GetEventDensity(integralPoint, currentTime) *
                                     (neighborEffects * SensingModelFunction(D + r));
                             }
                                                        else if(Phat<0.5)
                                                        {
                                integralSum += smallDetectionBoost* delta1 * r * density.GetEventDensity(integralPoint, currentTime) *
                                     (neighborEffects * SensingModelFunction(D + r));
                                                        }
                                                        else
                                                        {

                             integralSum += delta1 * r * density.GetEventDensity(integralPoint, currentTime) *
                                     (neighborEffects * SensingModelFunction(D + r));
                                                        }*/
                            //continuous low detection boost
                            //integralSum += delta1 * r * density.GetEventDensity(integralPoint, currentTime) *(Phat0-Phat0*Phat0/2-(Phat-Phat*Phat/2));
                        }

                        //The calculation is true only when there is one robot
                        //Otherwise, we have to do numerical integration along a line
                        //dFdx2+=sign*SinTheta/D*(-d/sensingDecayFactor*SensingModelFunction(d+D)-1/sensingDecayFactor/sensingDecayFactor*(SensingModelFunction(d+D)-SensingModelFunction(D)));

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
            activeReflexVertices.clear(); //this container only holds ARV from one FG, so when we move to another FG, we need to empty it first.
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

        //TODO: assuming no obstacles here, add visibiity test later

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
        	obstacles.get(i5).GetMapBuildingImpactPoint(position,
        			point2.ExtendToInfinite(position, FOV_edge_point_left), FOV_edge_impact_point_left, false);
        	
        	obstacles.get(i5).GetMapBuildingImpactPoint(position,
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

		//active map building force
		point2 activeMapBuildingForce = GetActiveMapBuildingForceFrontier();
	
        // double alpha1 = EventDensity.WALL_DECAY_FACTOR;
        //double alpha1 = 1;//use this line instead of the above to disable the second term of the derivative (reflex vertices)

        dFdx = -dFdx + dFdx2 - this.communicationCostWeight * (dFdx4) -this.dataCollectionWeight*dFdx3- dFdx5 + activeMapBuildingForce.x;
        dFdy = -dFdy + dFdy2 - this.communicationCostWeight * (dFdy4) -this.dataCollectionWeight* dFdy3- dFdy5 + activeMapBuildingForce.y;
        // System.out.println("Node"+this.id+"\tdFdx1:"+(-dFdx)+"\t\tdFdy1:"+(-dFdy)+"\t\tdFdx2:"+(dFdx2)+"\t\tdFdy2:"+(dFdy2));

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

        heading = Math.atan2(dFdy, dFdx); //final implementation on the robot
        if (heading == 0)
        {
            speed = 0;
        }

        // System.out.println("heading:"+heading);
        //end vehicle handling
    
	}
	 synchronized double NeighborEffects(point2 samplePoint)
	    {
	        double neighborEffect = 1;

	       for (int i1 = 0; i1 < neighbors.size(); i1++)
	        {
	            if (point2.Dist(samplePoint, neighbors.get(i1).estimatedPosition) < this.sensingCutoffRange) //SYNC
	            //Assuming the boundary is convex, the following testing is redundant
	            //if (boundary.LineOfSight(samplePoint, neighbors.get(i1).estimatedPosition))
	            {
	                double alpha = 1;

	                //test if the neighbor can see samplePoint
	                for (int i2 = 0; i2 < obstacles.size(); i2++)
	                {

	                    if (!obstacles.get(i2).MapBuildingLineOfSight(samplePoint, neighbors.get(i1).estimatedPosition))
	                    {
	                        alpha = EventDensity.WALL_DECAY_FACTOR;
	                        break;
	                    }

	                }
	                neighborEffect *=
	                        (1 - alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).estimatedPosition)));
	            }

	        }

	        return neighborEffect; //missing probability by neighbors
	    }
	 
	 protected point2 GetActiveMapBuildingForceFrontier()
	 {
		 point2 activeMapBuildingForce = new point2(0,0);
		 if (inActiveMapBuildingMode)
		 {
			 point2 frontierPoint = new point2(0,0);

			 for(int i=0; i<sim.mapBuildingGrid.length;i++)
	     	{
	     		for(int j=0; j<sim.mapBuildingGrid[i].length;j++)
	     		{
	     			if(sim.mapBuildingGrid[i][j] == 3)
	     			{
	     				frontierPoint.x = (0.5+i)*sim.mapBuildingIncrement;
	     				frontierPoint.y = (0.5+j)*sim.mapBuildingIncrement;
	     				double dist = point2.Dist(frontierPoint, position)/1000;  // divide 1000 to make MapBuilding force dominant 
	     				activeMapBuildingForce.x += (1/dist/dist/dist/dist/dist/dist)*(frontierPoint.x-position.x);
	   				    activeMapBuildingForce.y += (1/dist/dist/dist/dist/dist/dist)*(frontierPoint.y-position.y);
	     			}
	     		}
	     	}
		 }
		 return activeMapBuildingForce;
	 }
	 
	 protected point2 GetActiveMapBuildingForce()
	 {
		 point2 activeMapBuildingForce = new point2(0,0);
		 boolean isOpenBoundaryFound = false;
		 point2 nearestOpenBoundary = new point2(-99999,-99999);
		 point2 previousVertex;
		 
		 if (inActiveMapBuildingMode)
		 {
			 for (int i = 0; i<obstacles.size();i++)
			 {
				 if(obstacles.get(i).discoveredVerticesNumber>0 && !obstacles.get(i).isAllVerticesDiscovered)
				 {
					 previousVertex = obstacles.get(i).vertices.get(obstacles.get(i).vertices.size()-1);

					 for (int j = 0; j<obstacles.get(i).vertices.size();j++)
					 {
						 if (obstacles.get(i).vertices.get(j).isDiscovered != previousVertex.isDiscovered)
						 {
							 if(!isOpenBoundaryFound)
							 {
								 isOpenBoundaryFound = true;
								 nearestOpenBoundary = point2.MiddlePoint(obstacles.get(i).vertices.get(j),previousVertex);
							 }
							 else
							 {
								 point2 newOpenBoundary = point2.MiddlePoint(obstacles.get(i).vertices.get(j),previousVertex);
								 if(point2.Dist(newOpenBoundary, this.position)<point2.Dist(nearestOpenBoundary, this.position))
								 {
									 nearestOpenBoundary = newOpenBoundary;
								 }
							 }	 
						 }	
						 previousVertex = obstacles.get(i).vertices.get(j);
					 }
				 }
			 }
			 
			 if(isOpenBoundaryFound)
			 {
				 activeMapBuildingForce.x = 1000*(nearestOpenBoundary.x-position.x);
				 activeMapBuildingForce.y = 1000*(nearestOpenBoundary.y-position.y);
			 }
		 }
		 
		 return activeMapBuildingForce;
	 }

}
