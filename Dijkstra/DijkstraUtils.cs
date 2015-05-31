using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteractionDetection.Dijkstra
{
    static class DijkstraUtils
    {

        static public float GetInternodeTraversalCost(int nodeA, int nodeB)
        {
            Point pointA = GlobUtils.GetPoint(nodeA);
            Point pointB = GlobUtils.GetPoint(nodeB);

            int xA = pointA.x;
            int yA = pointA.y;
            int xB = pointB.x;
            int yB = pointB.y;

            if (Math.Abs(xA-xB) > 1 || Math.Abs(yA - yB)>1)
            {
                return float.MaxValue;
            }
            //Console.WriteLine(GlobUtils.GetEuclideanDistance(nodeA, nodeB));
            return GlobUtils.GetEuclideanDistance(nodeA, nodeB);
        }

        //static public float GetInternodeTraversalCost2(int nodeA, int nodeB)
        //{
        //    Point a = GlobUtils.GetPoint(nodeA);
        //    Point b = GlobUtils.GetPoint(nodeB);

        //    if (nodeA<nodeB)
        //    {
        //        int diffX = b.x - a.x;
        //        int diffY = b.y - a.y;

        //        GlobVar.AdjacancyList[nodeA][]

        //    }
            
        //}

        //public static IEnumerable<int> GetNearbyNodes(int centerNode)
        //{
        //    Point p = GlobUtils.GetPoint(centerNode);

        //    for (int i = -1; i < 2; i++)
        //    {
        //        for (int j = -1; j < 2; j++)
        //        {
        //            int neighbourIndex = GlobUtils.GetIndexNoBoundaryCheck(p.x + j, p.y + i);
        //            if (GlobUtils.BoundaryCheck(neighbourIndex))
        //            {
        //                yield return neighbourIndex;
        //            }
        //        }
        //    }
        //}


    }
}
