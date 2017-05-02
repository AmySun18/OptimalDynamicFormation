package coverage;

import java.util.ArrayList;

public class Obstacle
{
    int id;
    public ArrayList<point2> vertices;
    public ArrayList<point2> originalVertices;//replace the filled version after all the vertices are discovered to help performance
    point2 interiorPoint; //used for sight blocking or normal vector side determination

    double smallestX;
    double smallestY;
    double largestX; //Usually used for scanning, give a sense of the the size of the shape
    double largestY;
    
    public int discoveredVerticesNumber = 0;
    public boolean isAllVerticesDiscovered = false; //If all vertices of it is discovered, this obstacle is discovered

    public Obstacle()
    {
        vertices = new ArrayList<point2>();
    }

    //The ARVs returned are still only candidates. They could be blocked by other FGs. If they are verified to be true ARVs, they each will have an impact point on the boundary or other FGs
    void GetActiveReflexVertices(point2 observerPosition, ArrayList<point2> listARV)
    { 
        for (int i = 0; i < vertices.size(); i++)
        {
            if (LineOfSightMinusOneVertex(observerPosition, vertices.get(i), vertices.get(i)))
            { //the observer can see this vertex
                if (LineOfSightMinusOneVertex(point2.ExtendToInfinite(observerPosition, vertices.get(i)), vertices.get(i), vertices.get(i)))
                {
                    //The line connecting observer and the vertex doesn't cut through the shape
                    listARV.add(vertices.get(i));
                }
            }
        }
    }
    
    //The ARVs returned are still only candidates. They could be blocked by other FGs. If they are verified to be true ARVs, they each will have an impact point on the boundary or other FGs
    void GetMapBuildingActiveReflexVertices(point2 observerPosition, ArrayList<point2> listARV)
    {
        for (int i = 0; i < vertices.size(); i++)
        {
        	if (vertices.get(i).isDiscovered)
        	{
	            if (LineOfSightMinusOneVertex(observerPosition, vertices.get(i), vertices.get(i)))
	            { //the observer can see this vertex
	                if (LineOfSightMinusOneVertex(point2.ExtendToInfinite(observerPosition, vertices.get(i)), vertices.get(i), vertices.get(i)))
	                {
	                    //The line connecting observer and the vertex doesn't cut through the shape
	                    listARV.add(vertices.get(i));
	                }
	            }
        	}
        }
    }

    //Assuming the obstacles are all convex
	void updateInteriorPoint()
	{
	    point2 sum = new point2(0, 0);
	
	    for (int i = 0; i < vertices.size(); i++)
	    {
	        sum = point2.plus(sum, vertices.get(i));
	    }
	    interiorPoint = point2.divide(sum, (double) vertices.size());
	}

	boolean LineOfSightMinusOneVertex(point2 p1, point2 p2, point2 IgnoredVertex)
    {     
        {
            point2 previousPoint = vertices.get(vertices.size() - 1);
           
            for (int i = 0; i < vertices.size(); i++)
            {
                if (vertices.get(i) == IgnoredVertex || previousPoint == IgnoredVertex)
                {
                    previousPoint = (vertices.get(i));
                }
                else
                {
                    if (HasIntersection(p1, p2, vertices.get(i), previousPoint))
                    {
                        return false;
                    }
                    else
                    {
                        previousPoint = vertices.get(i);
                    }
                }
            }
            return true;
        }

    }

    final boolean LineOfSight(point2 p1, point2 p2)
    {
        //bounding box testing;
        if (p1.x > largestX && p2.x > largestX)
        {
            return true;
        }
        if (p1.y > largestY && p2.y > largestY)
        {
            return true;
        }
        if (p1.x < smallestX && p2.x < smallestX)
        {
            return true;
        }
        if (p1.y < smallestY && p2.y < smallestY)
        {
            return true;
        }

        if (originalVertices == null || originalVertices.size() == 0)
        { //It's empty, so it can block nothing.
             return true;
        }
        else
        {
            point2 previousPoint = originalVertices.get(originalVertices.size() - 1);
            for (int i = 0; i < originalVertices.size(); i++)
            {
                if (HasIntersection(p1, p2, originalVertices.get(i), previousPoint))
                {
                    return false;
                }
                else
                {
                    previousPoint = originalVertices.get(i);
                }
            }

            return true;
        }
    }
    
    
    public int noLineOfSightVerticeNum(point2 p1, point2 p2)
    {

            point2 previousPoint = originalVertices.get(originalVertices.size() - 1);
            for (int i = 0; i < originalVertices.size(); i++)
            {
                if (HasIntersection(p1, p2, originalVertices.get(i), previousPoint))
                {
                    return i;
                }
                else
                {
                    previousPoint = originalVertices.get(i);
                }
            }

            return -1;
     }
    

    boolean MapBuildingLineOfSight(point2 p1, point2 p2)
    {
    	  //bounding box testing;
        if (p1.x > largestX && p2.x > largestX)
        {
            return true;
        }
        if (p1.y > largestY && p2.y > largestY)
        {
            return true;
        }
        if (p1.x < smallestX && p2.x < smallestX)
        {
            return true;
        }
        if (p1.y < smallestY && p2.y < smallestY)
        {
            return true;
        }

          if (vertices.size() <= 1 || discoveredVerticesNumber<=1)
          { //It's empty or not discovered, so it can not block anything.
               return true;
          }
          else
        {
            point2 previousPoint = vertices.get(vertices.size() - 1);
            for (int i = 0; i < vertices.size(); i++)
            {
                if (vertices.get(i).isDiscovered && previousPoint.isDiscovered && HasIntersection(p1, p2, vertices.get(i), previousPoint))
                {
                    return false;
                }
                else
                {
                    previousPoint = vertices.get(i);
                }
            }

            return true;
        }
    }
    
    //Overloaded P2 TODO: not the best implementation
    //"direction" is used to return a value actually
    //used only in Robot's stateupdate routine for collision avoidance and finding a sliding direction
    boolean LineOfSight(point2 p1, point2 p2, point2 direction)
    {
        //bounding box testing; seems to be a cut the cost to 60%
        if (p1.x > largestX && p2.x > largestX)
        {
            return true;
        }
        if (p1.y > largestY && p2.y > largestY)
        {
            return true;
        }
        if (p1.x < smallestX && p2.x < smallestX)
        {
            return true;
        }
        if (p1.y < smallestY && p2.y < smallestY)
        {
            return true;
        }

         if (vertices.size() == 0)
         { //It's empty, so it can block nothing.
            return true;
         }
         else
        {       
        	//TODO: a line can actually intersect with an obstacle in two places. We should return the closest direction
        	//instead of the first one detected.
            point2 previousPoint = vertices.get(vertices.size() - 1);
            for (int i = 0; i < vertices.size(); i++)
            {
                if (HasIntersection(p1, p2, vertices.get(i), previousPoint))
                {
                    //direction = point2.minus(vertices.get(i), previousPoint);
                    direction.x = point2.minus(vertices.get(i), previousPoint).x;
                    direction.y = point2.minus(vertices.get(i), previousPoint).y;
                    return false;
                }
                else
                {
                    previousPoint = vertices.get(i);
                }
            }

            return true;
        }
    }

    void updateBoundingBox()
    {
        smallestX = largestX = vertices.get(0).x;
        smallestY = largestY = vertices.get(0).y;
        for (int i = 1; i < vertices.size(); i++)
        {
            if (vertices.get(i).x < smallestX)
            {
                smallestX = vertices.get(i).x;
            }
            if (vertices.get(i).y < smallestY)
            {
                smallestY = vertices.get(i).y;
            }
            if (vertices.get(i).y > largestY)
            {
                largestY = vertices.get(i).y;
            }
            if (vertices.get(i).x > largestX)
            {
                largestX = vertices.get(i).x;
            }

        }

    }

    //TODO: simplify it. currently use with caution
    boolean IsInteriorPoint(point2 testPoint)
    {
        int intersectionPoints = 0;
        point2 previousPoint = vertices.get(vertices.size() - 1);
        for (int i = 0; i < vertices.size(); i++)
        {
            //just make sure point2(-9999,-9999) is not inside of any polygon, it is a faraway point
            //Sometimes the test line falls on a vertex, the test result is wrong
            if (HasIntersection(testPoint, new point2( -99999.1234, -99999.4321), vertices.get(i), previousPoint))
            {
                intersectionPoints++;
            }
            previousPoint = vertices.get(i);
        }

        if (intersectionPoints % 2 == 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }


    //Helper function for HasIntersection()
    private final boolean OnSegment(point2 a, point2 b, point2 c)
    {
        if ((Math.min(a.x, b.x) <= c.x && c.x <= Math.max(a.x, b.x)) && (Math.min(a.y, b.y) <= c.y && c.y <= Math.max(a.y, b.y)))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //Line segment intersection test, cool stuff
    //Solution
    private final boolean HasIntersection(point2 p1, point2 p2, point2 p3, point2 p4) //using final might help performance, I am not sure
    {
 
        double d1 = (p1.x - p3.x) * (p4.y - p3.y) - (p1.y - p3.y) * (p4.x - p3.x);
        double d2 = (p2.x - p3.x) * (p4.y - p3.y) - (p2.y - p3.y) * (p4.x - p3.x);
        double d3 = (p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x);
        double d4 = (p4.x - p1.x) * (p2.y - p1.y) - (p4.y - p1.y) * (p2.x - p1.x);

        // double d1 = point2.cross(new point2(p1.x-p3.x,p1.y-p3.y),new point2(p4.x-p3.x,p4.y-p3.y));
        // double d2 = point2.cross(new point2(p2.x-p3.x,p2.y-p3.y),new point2(p4.x-p3.x,p4.y-p3.y));
        // double d3 = point2.cross(new point2(p3.x-p1.x,p3.y-p1.y),new point2(p2.x-p1.x,p2.y-p1.y));
        // double d4 = point2.cross(new point2(p4.x-p1.x,p4.y-p1.y),new point2(p2.x-p1.x,p2.y-p1.y));

        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
        {
            return true;
        }
        else if ((d1 == 0) && OnSegment(p3, p4, p1))
        {
            return true;
        }
        else if ((d2 == 0) && OnSegment(p3, p4, p2))
        {
            return true;
        }
        else if ((d3 == 0) && OnSegment(p1, p2, p3))
        {
            return true;
        }
        else if ((d4 == 0) && OnSegment(p1, p2, p4))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //Line-line intersection point
    point2 GetIntersection(point2 p1, point2 p2, point2 p3, point2 p4)
    {
        //if two lines are parallel, the return value will be NaN and Infinite
        //the line segments (p1,p2) and (p3,p4) do not have to intersect. The lines defined by them will intersect if not parallel.
        return new point2(((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p3.x * p4.y - p3.y * p4.x) * (p1.x - p2.x)) /
                          ((p1.x - p2.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p2.y)),
                          ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p3.x * p4.y - p3.y * p4.x) * (p1.y - p2.y)) /
                          ((p1.x - p2.x) * (p3.y - p4.y) - (p3.x - p4.x) * (p1.y - p2.y)));
    }

    //return true if there is a better impact point candidate, better in the sense that it's closer to the startPoint
    //IMPORTANT: Before call this function for the first time, make sure CurrentBestImpactPoint initialized to be very far away.
    boolean GetImpactPoint(point2 startPoint, point2 farawayPoint, point2 CurrentBestImpactPoint, boolean Uninitialized)
    {
        point2 impactPointCandidate;
        boolean hasBetterImpactCandidate = false;
        point2 previousPoint = vertices.get(vertices.size() - 1);
        for (int i = 0; i < vertices.size(); i++)
        {
            if (HasIntersection(startPoint, farawayPoint, vertices.get(i), previousPoint))
            {
                impactPointCandidate = GetIntersection(startPoint, farawayPoint, vertices.get(i), previousPoint);
                if (Uninitialized)
                {
                    CurrentBestImpactPoint.x = impactPointCandidate.x;
                    CurrentBestImpactPoint.y = impactPointCandidate.y;
                    hasBetterImpactCandidate = true;
                    Uninitialized = false;  //TODO: changing the value of the method parameter, is the change on Uninitialized passed back to the caller? 
                }
                else
                {
                    if (point2.Dist(impactPointCandidate, startPoint) < point2.Dist(CurrentBestImpactPoint, startPoint))
                    {
                        CurrentBestImpactPoint.x = impactPointCandidate.x;
                        CurrentBestImpactPoint.y = impactPointCandidate.y;
                        hasBetterImpactCandidate = true;
                    }
                }
            }
            //else
            // {
            previousPoint = vertices.get(i);
            // }
        }
        return hasBetterImpactCandidate;
    }
    
    //return true if there is a better impact point candidate, better in the sense that it's closer to the startPoint
    //IMPORTANT: Before call this function for the first time, make sure CurrentBestImpactPoint initialized to be very far away.
    boolean GetMapBuildingImpactPoint(point2 startPoint, point2 farawayPoint, point2 CurrentBestImpactPoint, boolean Uninitialized)
    {
        point2 impactPointCandidate;
        boolean hasBetterImpactCandidate = false;
        
        if (vertices.size() <= 1 || discoveredVerticesNumber<=1)
        { //It's empty or not discovered, so no impact point can be on it.
             return false;
        }
        else
        {
        point2 previousPoint = vertices.get(vertices.size() - 1);
        for (int i = 0; i < vertices.size(); i++)
        {
            if (vertices.get(i).isDiscovered && previousPoint.isDiscovered && HasIntersection(startPoint, farawayPoint, vertices.get(i), previousPoint))
            {
                impactPointCandidate = GetIntersection(startPoint, farawayPoint, vertices.get(i), previousPoint);
                if (Uninitialized)
                {
                    CurrentBestImpactPoint.x = impactPointCandidate.x;
                    CurrentBestImpactPoint.y = impactPointCandidate.y;
                    hasBetterImpactCandidate = true;
                    Uninitialized = false;  //TODO: changing the value of the method parameter, is the change on Uninitialized passed back to the caller? 
                }
                else
                {
                    if (point2.Dist(impactPointCandidate, startPoint) < point2.Dist(CurrentBestImpactPoint, startPoint))
                    {
                        CurrentBestImpactPoint.x = impactPointCandidate.x;
                        CurrentBestImpactPoint.y = impactPointCandidate.y;
                        hasBetterImpactCandidate = true;
                    }
                }
            }
            //else
            // {
            previousPoint = vertices.get(i);
            // }
        }
        }
        return hasBetterImpactCandidate;
    }

    double IntersectionSegmentGradient(point2 movingPoint, point2 stationaryPoint, point2 returnGradient)
    {
        // returnGradient = new point2(0, 0);

        //use finite difference for now
        double delta = 0.0001; //small difference

        double b = BlockDistance(movingPoint,stationaryPoint);
        returnGradient.x = ( BlockDistance(point2.plus(movingPoint, new point2(delta, 0)),stationaryPoint)- b) / delta;
        returnGradient.y = (BlockDistance(point2.plus(movingPoint, new point2(0, delta)),stationaryPoint) - b) / delta;
        return b;  //return the blockDistance

    }

    public void fillinVertices()
    {
    	this.originalVertices =(ArrayList<point2>)this.vertices.clone();
    	
    	double fillinInterval = 1; //TODO: was 2, make it adjustable in UI
    	ArrayList<point2> newVertices = new ArrayList<point2>();
    	//point2 slidingPointOnEdge = new point2();
    	 for (int i = 0; i < vertices.size()-1; i++)
         {
    		 newVertices.add(vertices.get(i));
    		 double verticeDist = point2.Dist(vertices.get(i), vertices.get(i+1));
    		 double horizontalVerticesDist = (vertices.get(i+1).x - vertices.get(i).x)/verticeDist;
    		 double verticalVerticesDist= (vertices.get(i+1).y - vertices.get(i).y)/verticeDist;
    		 int pointsOnEdge = (int)Math.ceil(verticeDist/fillinInterval)-1;
    		 for (int i1 = 1; i1 <= pointsOnEdge; i1++)
             {
    			 newVertices.add(new point2(i1*fillinInterval*horizontalVerticesDist+vertices.get(i).x,i1*fillinInterval*verticalVerticesDist+vertices.get(i).y));
             }
    		 
         }   	 
    	 //special case for the last point
    	 newVertices.add(vertices.get(vertices.size()-1));
		 double verticeDist = point2.Dist(vertices.get(vertices.size()-1), vertices.get(0));
		 double horizontalVerticesDist = (vertices.get(0).x - vertices.get(vertices.size()-1).x)/verticeDist;
		 double verticalVerticesDist= (vertices.get(0).y - vertices.get(vertices.size()-1).y)/verticeDist;
		 int pointsOnEdge = (int)Math.floor(verticeDist/fillinInterval)-1;
		 for (int i1 = 1; i1 <= pointsOnEdge; i1++)
         {
			 newVertices.add(new point2(i1*fillinInterval*horizontalVerticesDist+vertices.get(vertices.size()-1).x,i1*fillinInterval*verticalVerticesDist+vertices.get(vertices.size()-1).y));
         }
    	 
    	 vertices = newVertices;
    }
    
    double BlockDistance(point2 p1, point2 p2)
    {
        //if obstacles are convex, there could be at most 2 intersection points, unless (p1,p2) coincide with an edge which is extremely unlikely
        point2 firstIntersection = new point2(0, 0);
        point2 secondIntersection = new point2(0, 0);

        boolean firstIntersectionFound = false;

        point2 previousPoint = vertices.get(vertices.size() - 1);
        for (int i = 0; i < vertices.size(); i++)
        {
            if (HasIntersection(p1, p2, vertices.get(i), previousPoint))
            {
                if (!firstIntersectionFound)
                {
                    firstIntersection = GetIntersection(p1, p2, vertices.get(i), previousPoint);
                    firstIntersectionFound = true;
                }

                else
                {
                    secondIntersection = GetIntersection(p1, p2, vertices.get(i), previousPoint);
                    break;
                }
            }
            previousPoint = vertices.get(i);
        }
        return point2.Dist(firstIntersection, secondIntersection);
    }

}

//example mission space
/*
17 15 17 35 43 35 43 15
28 -0.1 28.5 13 31.5 13 32 -0.1
28.5 38 28 50.1 32 50.1 31.5 38
-0.1 23 -0.1 27 13.5 26.5 13.5 23.5
45 23 45 27 60.1 27 60.1 23

*********************************
for target tracking
13 15 13 50 20 50 20 15
30 7 30 18 33 28 37 28 37 7
32 35 32 45 43 45 43 35
46 10 46 18 54 18 54 10

*/

/**********************************************
5 -1 5 20 7 20 7 -1
15 -1 15 20 17 20 17 -1
25 -1 25 20 27 20 27 -1
35 -1 35 20 37 20 37 -1
45 -1 45 20 47 20 47 -1
7 26 7 51 8 51 8 26
27 26 27 51 28 51 28 26
47 26 47 51 48 51 48 26
57 26 57 51 58 51 58 26

*********************************************
7.5 7.5 2.5 12.5 7.5 17.5 12.5 12.5
7.5 20 2.5 25 7.5 30 12.5 25
7.5 32.5 2.5 37.5 7.5 42.5 12.5 37.5
22.5 7.5 17.5 12.5 22.5 17.5 27.5 12.5
22.5 20 17.5 25 22.5 30 27.5 25
22.5 32.5 17.5 37.5 22.5 42.5 27.5 37.5
32.5 7.5 32.5 17.5 42.5 17.5 42.5 7.5
32.5 20 32.5 30 42.5 30 42.5 20
32.5 32.5 32.5 42.5 42.5 42.5 42.5 32.5
47.5 7.5 47.5 17.5 57.5 17.5 57.5 7.5
47.5 20 47.5 30 57.5 30 57.5 20
47.5 32.5 47.5 42.5 57.5 42.5 57.5 32.5

******************************************
5 5 5 20 7 20 7 5
4 30 4 32 20 32 20 30
15 10 15 12 26 12 26 10
40 8 40 20 50 20 50 8
32 35 32 47 35 47 35 35
36 32 36 34 52 34 52 32

******************************************

9 -0.1 9 41 11 41 11 -0.1
11 39 11 41 51 41 51 39
49 9 49 39 51 39 51 9
19 9 19 11 51 11 51 9
19 9 19 31 21 31 21 9
19 29 19 31 41 31 41 29
39 19 39 31 41 31 41 19
29 19 29 21 41 21 41 19
******************************************
9 -0.01 9 41 11 41 11 -0.01
11.001 39 11.001 41 51 41 51 39
49 11.001 49 38.999 51 38.999 51 11.001
21.001 9 21.001 11 51 11 51 9
19 9 19 31 21 31 21 9
21.001 29 21.001 31 38.999 31 38.999 29
39 19 39 31 41 31 41 19
29 19 29 21 38.999 21 38.999 19
**************************************
15 25 15 35 25 40 25 30
30 10 30 15 40 25 40 20

***********************************
30 20 30 25 50 25 50 20
30 40 30 45 50 45 50 40

*********************************
25 15  30 20 40 20 45 15 30 15
30 30 25 35 45 35 40 30


*********************************************/
