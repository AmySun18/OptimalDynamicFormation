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


public class Aggregator
{
    static final boolean DEBUG = false;
    Obstacle boundary;
    ArrayList<Obstacle> obstacles;
    EventDensity density;
   public ArrayList<Element> currentElements;
   public   ArrayList<Element> coreElements;
   public double attractionRange = 15;

    public Aggregator(Obstacle bndy, ArrayList<Obstacle> obstls, EventDensity dnst)
    {
        boundary = bndy;
        obstacles = obstls;
        density = dnst;
        currentElements = new ArrayList<Element>();
        coreElements  = new ArrayList<Element>();
    }

    void CreateElements()
    {
        currentElements.clear(); //aggregation might be used multiple times and obstacles might be changed
        double creation_increment = 2;
        for (double creation_x = 1; creation_x < boundary.largestX; creation_x += creation_increment)
        {
            for (double creation_y = 1; creation_y < boundary.largestY; creation_y += creation_increment)
            {
                boolean skip_creation = false;
                for (int i = 0; i < obstacles.size(); i++)
                // for (list<WorldGeometry>::iterator glassWallsIterator=glassWalls.begin();glassWallsIterator!=glassWalls.end();glassWallsIterator++)
                {

                    if (obstacles.get(i).LineOfSight(new point2(creation_x, creation_y),obstacles.get(i).interiorPoint))
                    {
                        if(DEBUG)
                        {
                            System.out.println("Skiped:" + creation_x + "," + creation_y);
                        }
                        skip_creation = true;
                        break;
                    }
                }

                if (!skip_creation)
                {
                    currentElements.add(new Element(new point2(creation_x, creation_y), density.GetEventDensity(new point2(creation_x, creation_y))));
                }
            }
        }
    }

//return true if more iteration is needed.
    boolean AggregationOneStep()
    {

        point2 distance = new point2();
        double length;
        boolean blocked;
        for (int i1 = 0; i1 < currentElements.size(); i1++)
        {
            currentElements.get(i1).heading.x = 0;
            currentElements.get(i1).heading.y = 0;
            for (int i2 = 0; i2 < currentElements.size(); i2++)
            {
               distance.x = currentElements.get(i2).position.x-currentElements.get(i1).position.x;
                distance.y = currentElements.get(i2).position.y-currentElements.get(i1).position.y;
                 length = distance.Length();
                blocked = false;
                if (length> 1.5 && length < attractionRange)
                {
                    if (!boundary.LineOfSight(currentElements.get(i1).position, currentElements.get(i2).position))
                    {
                        blocked = true;
                    }
                    else
                    {
                        for (int i3 = 0; i3 < obstacles.size(); i3++)

                        {
                            if (!(obstacles.get(i3).LineOfSight(currentElements.get(i1).position, currentElements.get(i2).position)))
                            {
                                blocked = true;
                                break;
                            }
                        }
                    }

                    if (!blocked)
                    {
                            distance.x = 0.2 * distance.x / Math.pow(length, 3);
                            distance.y = 0.2 * distance.y / Math.pow(length, 3);
                            currentElements.get(i1).heading.x = currentElements.get(i1).heading.x+ distance.x; //0.2 is a tentative scaling factor, pow(3) is also experimental
                    currentElements.get(i1).heading.y = currentElements.get(i1).heading.y+ distance.y;
                        }
                }
            }
        }

        boolean needMoreIteration = false;

        //for (list<Element>::iterator ElementIterator = currentElements.begin(); ElementIterator!=currentElements.end();ElementIterator++)
        for (int i1 = 0; i1 < currentElements.size(); i1++)
        {
            if (currentElements.get(i1).heading.Length() > 1.0)
            {
                if(DEBUG)
                        {
                System.out.println("catch a bad guy "+currentElements.get(i1).heading.Length());
                        }
                currentElements.get(i1).heading.x = currentElements.get(i1).heading.x / currentElements.get(i1).heading.Length() * 1.0;
                currentElements.get(i1).heading.y = currentElements.get(i1).heading.y / currentElements.get(i1).heading.Length() * 1.0;
            }

            if (currentElements.get(i1).heading.Length() > 0)
            {
                currentElements.get(i1).position.x = currentElements.get(i1).position.x + currentElements.get(i1).heading.x;
                currentElements.get(i1).position.y = currentElements.get(i1).position.y + currentElements.get(i1).heading.y;
                needMoreIteration = true;
            }
        }
        return needMoreIteration;
    }


    void ElementAggregation()
    {
      /*  ofstream ElementHistoryOutput;
        ElementHistoryOutput.open("D:\\public\\ElementHistory.txt");
        for (int i = 0; i < 1000; i++) //number of iteration is an upperbound, it should break long before that
        {
            if (AggregationOneStep())
            {
                for (list<Element> : : iterator ElementIterator = currentElements.begin(); ElementIterator != currentElements.end();
                        ElementIterator++)
                {
                    ElementHistoryOutput.width(8);
                    ElementHistoryOutput << ( * ElementIterator).position.x << "\t";
                    ElementHistoryOutput << ( * ElementIterator).position.y << "\t";
                    //ElementHistoryOutput<<(*ElementIterator).heading.x<<"\t";
                    //ElementHistoryOutput<<(*ElementIterator).heading.y<<"\t";
                }
                ElementHistoryOutput << "\n";
                DP1("%d\n", i);
            }
            else
            {
                break;
            }
        }
        ElementHistoryOutput.close();*/
    }

    void coreElementsUpdate()
    {
        //Calculate cores and their weights
       coreElements.clear();

       for (int i1 = 0; i1 < currentElements.size(); i1++)
       //for (list<Element> : : iterator ElementIterator = currentElements.begin(); ElementIterator != currentElements.end(); ElementIterator++)
       {
           boolean fresh_core = true;
           for (int i2 = 0; i2 < coreElements.size(); i2++)
           //for (list<Element> : : iterator ElementIteratorInner = coreElements.begin(); ElementIteratorInner != coreElements.end();               ElementIteratorInner++)
           {

               if (point2.Dist(currentElements.get(i1).position,  coreElements.get(i2).position) <= 1.5)
               {
                   coreElements.get(i2).weight += currentElements.get(i1).weight;
                   fresh_core = false;
                   break;
               }
           }
           if (fresh_core)
           {
               coreElements.add(currentElements.get(i1));
           }
       }

       /*      coreElements.sort();
       ofstream CoreElementsOutput;
       CoreElementsOutput.open("D:\\public\\CoreElements.txt");
       for (list<Element> : : iterator ElementIterator = coreElements.begin(); ElementIterator != coreElements.end(); ElementIterator++)
       {
           CoreElementsOutput.width(8);
           CoreElementsOutput << ElementIterator - > position.x << "\t" << ElementIterator - > position.y << "\t" << ElementIterator - >
                   weight << "\n";
           System.out.println("Core at (%f,%f) weight: %f\n", ElementIterator - > position.x, ElementIterator - > position.y,
                              ElementIterator - > weight);
       }
       CoreElementsOutput.close();*/


    }

}


class Element
{

    public point2 position;
    public double weight;

    point2 heading = new point2(); //a vector relative to (0,0)
    Element(point2 pos, double w)
    {
        position = pos;
        weight = w;
    }
}
