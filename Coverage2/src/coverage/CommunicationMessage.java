package coverage;

public class CommunicationMessage {
	public double x,y;
	public double heading;
	public int remaining_delay;
	
	public CommunicationMessage(point2 p, double h, int init_delay)
	{
		x = p.x;
		y = p.y;
		heading = h;
		remaining_delay = init_delay;
	}

}
