package coverage;

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
public class Event
{
    public static double EVENT_DURATION = 80;  // the lifespan of an event, for now it is fixed.
    public static double EVENT_SPAWN_INTERVAL = 4; //determines how often events are generated.
    public point2 eventLocation = new point2();
    public double spawnTime;
    public double monitorPercentage = 0.0;

    public Event()
    {
    }

    public Event(double x, double y, double time)
    {
        eventLocation.x = x;
        eventLocation.y = y;
        spawnTime = time;
    }
}
