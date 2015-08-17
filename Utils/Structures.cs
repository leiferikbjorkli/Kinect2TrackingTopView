//
// Written by Leif Erik Bjoerkli
//


namespace InteractionDetection
{
    public struct ThreePointRectangle
    {
        public ThreePointRectangle(Point a, Point b, Point c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.CenterPoint = new Point(0,0);
        }
        public ThreePointRectangle(Point a, Point b, Point c,Point CenterPoint)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.CenterPoint = CenterPoint;
        }
        public Point a, b, c,CenterPoint;
    }
    public struct Point
    {
        public Point(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
        public int x, y;
    }

    public struct IndexRectangle
    {
        public IndexRectangle(int a, int b, int c)
        {
            this.a = a;
            this.b = b;
            this.c = c;
        }

        public int a, b, c;
    }

   
}
