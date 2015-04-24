using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using AForge.Imaging.Filters;
using AForge;
using AForge.Imaging;
using AForge.Math.Geometry;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Drawing;
using System.Drawing.Imaging;
using AForge.Vision.Motion;



namespace Microsoft.Samples.Kinect.DepthBasics
{
    public enum HighlightType
    {
        ConvexHull = 0,
        LeftAndRightEdges = 1,
        TopAndBottomEdges = 2,
        Quadrilateral = 3
    }


    public class BlobDetector
    {

        private BlobCounter blobCounter = null;
        private Blob[] blobs = null;


        Dictionary<int, List<IntPoint>> leftEdges = new Dictionary<int, List<IntPoint>>();
        Dictionary<int, List<IntPoint>> rightEdges = new Dictionary<int, List<IntPoint>>();
        Dictionary<int, List<IntPoint>> topEdges = new Dictionary<int, List<IntPoint>>();
        Dictionary<int, List<IntPoint>> bottomEdges = new Dictionary<int, List<IntPoint>>();

        Dictionary<int, List<IntPoint>> hulls = new Dictionary<int, List<IntPoint>>();
        Dictionary<int, List<IntPoint>> quadrilaterals = new Dictionary<int, List<IntPoint>>();

        public BlobDetector() 
        {
            this.blobCounter = new BlobCounter();
        }

        public BitmapSource ProcessImage(WriteableBitmap writeBitmap, VideoStreamSaver writer)
        {

            leftEdges.Clear();
            rightEdges.Clear();
            topEdges.Clear();
            bottomEdges.Clear();
            hulls.Clear();
            quadrilaterals.Clear();

            Bitmap depthBitmap = Helpers.writeableBitmapToBitmap(writeBitmap);




            writer.addFrame(depthBitmap);
                





            ////Blobcounter filter
            //blobCounter.BackgroundThreshold = System.Drawing.Color.FromArgb(230,230,230);
            //blobCounter.MaxHeight = 200;
            //blobCounter.MaxWidth = 200;
            //blobCounter.MinWidth = 100;
            //blobCounter.MinWidth = 100;
            //blobCounter.FilterBlobs = true;



            //blobCounter.ProcessImage(depthBitmap);
            //blobs = blobCounter.GetObjectsInformation();

            //GrahamConvexHull grahamScan = new GrahamConvexHull();

            //foreach(Blob blob in blobs){

            //    List<IntPoint> leftEdge = new List<IntPoint>();
            //    List<IntPoint> rightEdge = new List<IntPoint>();
            //    List<IntPoint> topEdge = new List<IntPoint>();
            //    List<IntPoint> bottomEdge = new List<IntPoint>();

            //    // collect edge points
            //    blobCounter.GetBlobsLeftAndRightEdges(blob, out leftEdge, out rightEdge);
            //    blobCounter.GetBlobsTopAndBottomEdges(blob, out topEdge, out bottomEdge);

            //    leftEdges.Add(blob.ID, leftEdge);
            //    rightEdges.Add(blob.ID, rightEdge);
            //    topEdges.Add(blob.ID, topEdge);
            //    bottomEdges.Add(blob.ID, bottomEdge);

            //    // find convex hull
            //    List<IntPoint> edgePoints = new List<IntPoint>();
            //    edgePoints.AddRange(leftEdge);
            //    edgePoints.AddRange(rightEdge);

            //    List<IntPoint> hull = grahamScan.FindHull(edgePoints);
            //    hulls.Add(blob.ID, hull);


            //    List<IntPoint> quadrilateral = null;

            //    // find quadrilateral
            //    if (hull.Count < 4)
            //    {
            //        quadrilateral = new List<IntPoint>(hull);
            //    }
            //    else
            //    {
            //        quadrilateral = PointsCloud.FindQuadrilateralCorners(hull); 
            //    }
            //    quadrilaterals.Add(blob.ID, quadrilateral);

                
            
            //}

            //depthBitmap = DrawHighlights(depthBitmap);

            //depthBitmap = detectCircles(depthBitmap);

            BitmapSource bitmapSource = Helpers.ToBitmapSource(depthBitmap);



            return bitmapSource;
        }


        private Bitmap detectCircles(Bitmap depthBitmap) {

            Bitmap temp = new Bitmap(depthBitmap.Width, depthBitmap.Height);
            Graphics g = Graphics.FromImage(temp);
            g.DrawImage(depthBitmap, new Rectangle(0, 0, depthBitmap.Width, depthBitmap.Height), 0, 0, depthBitmap.Width, depthBitmap.Height, GraphicsUnit.Pixel);

            System.Drawing.Pen bluePen = new System.Drawing.Pen(System.Drawing.Color.Blue, 2);

            SimpleShapeChecker shapeChecker = new SimpleShapeChecker();
            for (int i = 0; i < blobs.Length; i++)
            {
                List<IntPoint> edgePoints = blobCounter.GetBlobsEdgePoints(blobs[i]);

                float relativeDistortionLimit = 0.5f;
                float minAcceptableDistortion = 0.3f;

                IntPoint minXY, maxXY;
                PointsCloud.GetBoundingRectangle(edgePoints, out minXY, out maxXY);

                IntPoint cloudSize = maxXY - minXY;
                DoublePoint center = minXY + (DoublePoint)cloudSize / 2;
                float radius = ((float)cloudSize.X + cloudSize.Y) / 4;

                float meanDistance = 0;
                for (int j = 0; j < edgePoints.Count; j++)
                {
                    meanDistance += Math.Abs((float)center.DistanceTo(edgePoints[j]) - radius);
                }
                meanDistance /= edgePoints.Count;

                float maxDistance = Math.Max(minAcceptableDistortion,((float)cloudSize.X + cloudSize.Y)/2 * relativeDistortionLimit);

                if (meanDistance <= maxDistance && radius>20) {
                    g.DrawEllipse(bluePen,
                        (int)(center.X - radius),
                        (int)(center.Y - radius),
                        (int)(radius * 2),
                        (int)(radius * 2));
                }
                

            }

            return temp;
        
        }


        private Bitmap DrawHighlights(Bitmap depthBitmap) {

            Bitmap temp = new Bitmap(depthBitmap.Width, depthBitmap.Height);
            Graphics g = Graphics.FromImage(temp);
            g.DrawImage(depthBitmap, new Rectangle(0, 0, depthBitmap.Width, depthBitmap.Height), 0, 0, depthBitmap.Width, depthBitmap.Height, GraphicsUnit.Pixel);

            Rectangle rect = new Rectangle(0, 0, temp.Width, temp.Height);
            
            System.Drawing.Pen borderPen = new System.Drawing.Pen(System.Drawing.Color.FromArgb(255,255,0), 4);
            System.Drawing.Pen highlightPen = new System.Drawing.Pen(System.Drawing.Color.Red,3);
            System.Drawing.Pen highlightPenBold = new System.Drawing.Pen(System.Drawing.Color.FromArgb(0, 255, 0), 3);
            System.Drawing.Pen rectPen = new System.Drawing.Pen(System.Drawing.Color.Blue,3);

            g.DrawRectangle(borderPen, rect.X + 1, rect.Y + 1, rect.Width - 2, rect.Height - 2);

            int selectedBlobID = 0;
            bool showRectangleAroundSelection = true;

            if (temp != null)
            {
                foreach (Blob blob in blobs) {
                    
                    System.Drawing.Pen pen = (blob.ID == selectedBlobID) ? highlightPenBold : highlightPen;

                    if (showRectangleAroundSelection && blob.ID == selectedBlobID)
                    {
                        g.DrawRectangle(rectPen, blob.Rectangle);
                    }


                    HighlightType highlighting = HighlightType.LeftAndRightEdges;
                    switch (highlighting)
                    {
                        case HighlightType.ConvexHull:
                            g.DrawPolygon(pen, PointsListToArray(hulls[blob.ID]));
                            break;
                        case HighlightType.LeftAndRightEdges:
                            DrawEdge(g, pen, leftEdges[blob.ID]);
                            DrawEdge(g, pen, rightEdges[blob.ID]);
                            break;
                        case HighlightType.TopAndBottomEdges:
                            DrawEdge(g, pen, topEdges[blob.ID]);
                            DrawEdge(g, pen, bottomEdges[blob.ID]);
                            break;
                        case HighlightType.Quadrilateral:
                            g.DrawPolygon(pen, PointsListToArray(quadrilaterals[blob.ID]));
                            break;
                    }
                }
            }
            else
            {
                g.FillRectangle(new SolidBrush(System.Drawing.Color.FromArgb(128, 128, 128)), rect.X + 1, rect.Y + 1, rect.Width - 2, rect.Height - 2);
            }
            return temp;
        }

        // Convert list of AForge.NET's IntPoint to array of .NET's Point
        private static System.Drawing.Point[] PointsListToArray(List<IntPoint> list)
        {
            System.Drawing.Point[] array = new System.Drawing.Point[list.Count];

            for (int i = 0, n = list.Count; i < n; i++)
            {
                array[i] = new System.Drawing.Point(list[i].X, list[i].Y);
            }

            return array;
        }

        // Draw object's edge
        private static void DrawEdge(Graphics g, System.Drawing.Pen pen, List<IntPoint> edge)
        {
            System.Drawing.Point[] points = PointsListToArray(edge);

            if (points.Length > 1)
            {
                g.DrawLines(pen, points);
            }
            else
            {
                g.DrawLine(pen, points[0], points[0]);
            }
        }

        private static Bitmap noiseFilter(Bitmap depthBitmap) {

            int threads = AForge.Parallel.ThreadsCount;
            threads = 3;
            int blockHeight = depthBitmap.Height/threads;
            List<Bitmap> blockList = new Bitmap[threads-1].ToList();


            for (int i = 0; i < threads; i++)
            {
                Bitmap temp = new Bitmap(depthBitmap);
                Rectangle rect = new Rectangle(0,blockHeight*i, depthBitmap.Width,blockHeight);
                Bitmap bitmapBlock = temp.Clone(rect, temp.PixelFormat);
                blockList.Insert(i,bitmapBlock);
            }

            AForge.Parallel.For(0, blockList.Count, delegate(int i)
            {
                ImageFilters filter = new ImageFilters();
                Bitmap filteredBitmap = filter.MedianFilter(blockList[i], 3, 0, true);
                blockList.Insert(i, filteredBitmap);
            });

            //indexed
            System.Drawing.Graphics g = System.Drawing.Graphics.FromImage(depthBitmap);

            for (int i = 0; i < threads; i++)
            {
                g.DrawImage(blockList[i], 0, blockHeight * i, depthBitmap.Width, depthBitmap.Height);
            }


            return depthBitmap;
        
        }


    }
}
