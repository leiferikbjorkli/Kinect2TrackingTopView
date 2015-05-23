using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Diagnostics;

namespace InteractionDetection
{




    public static class ImageUtils
    {
        // Optimization: Dont pass large variable to methods, allocate and reclaim new objects quickly, lists 7 times slower than arrays.
        // Measure 
        // Possible to spawn own background thread 
        // Label components recursively

        //private static List<PixelEdge> pixelEdges = new List<PixelEdge>();
        
        //private static PixelVertex[] vertices = new PixelVertex[frameWidth*frameHeight];

        private const int frameWidth = 512;
        private const int frameHeight = 424;
        private const int frameLength = frameWidth * frameHeight;
        private const int scaledFrameWidth = frameWidth / 2;
        private const int scaledFrameHeight = frameHeight / 2;
        private const int scaledFrameLength = scaledFrameHeight * scaledFrameWidth;

        private const int numVertices = frameWidth * frameHeight;

        private static Edge[] edges = new Edge[frameHeight * frameWidth * 4];
        private static float[] threshold = new float[numVertices];

        //private static PixelVertex[] pixelVertices = new PixelVertex[frameWidth*frameHeight];

        //private static PixelEdge[] pixelEdges = new PixelEdge[(frameHeight-2) * (frameWidth-1)*4];

        //private static List<PixelEdge> pixelEdges = new List<PixelEdge>();


        private static int CompareCameraSpacePoints(CameraSpacePoint p1, CameraSpacePoint p2)
        {
            return p1.Z.CompareTo(p2.Z);
        
        }

        //sort, define threshold from nr of people public static unsafe void
        //HysteresisThresholding(int numPeople, CameraSpacePoint[] pointCloud) {
        //    //CameraSpacePoint[] filteredFrame = new CameraSpacePoint[GlobalVariables.scaledFrameLength];
        //    Stopwatch sw = new Stopwatch();
        //    sw.Start();
        //    float[] depthMap = CreateDepthMapFromPointCloud(pointCloud);
        //    float minHeight = depthMap.Min();
        //    int approxHeadSize = 90;
        //    //int countBiggerPixels = 0;
        //    sw.Stop();
        //    Console.WriteLine("Hysteresis: {0}",sw.ElapsedMilliseconds);


        //}

        //public static unsafe void SegmentGraph(float c, CameraSpacePoint[] pointCloud) 
        //{
        //    float[] depthMap = CreateDepthMapFromPointCloud(pointCloud);

        //    Stopwatch sw = new Stopwatch();
        //    sw.Start();
        //    List<Edge> edgeList = new List<Edge>();
        //    edgeList = createComponentEdgesList(depthMap, edgeList);
        //    edgeList.Sort();
        //    sw.Stop();
        //    Console.WriteLine("List: {0}", sw.ElapsedMilliseconds);
        //    int numEdgesList = edgeList.Count;

        //    SegmentationSet segmentation = new SegmentationSet(numVertices);
            
        //    fixed (float* pThreshold = threshold)
        //    {
        //        for (int i = 0; i < numVertices; i++)
        //        {
        //            pThreshold[i] = c / 1;
        //        }
        //    }

        //    Stopwatch sw3 = new Stopwatch();
        //    sw3.Start();
        //    for (int i = 0; i < numEdgesList; i++)
        //    {
        //        Edge currentEdge = edgeList.ElementAt(i);
        //        int firstComp = currentEdge.firstComponent;
        //        int secondComp = currentEdge.secondComponent;
        //        float weight = currentEdge.weight;

        //        int firstCompIndex = segmentation.find(firstComp);
        //        int secondCompIndex = segmentation.find(secondComp);

        //        if (firstCompIndex != secondCompIndex)
        //        {
        //            if ((weight <= threshold[firstCompIndex]) && (weight <= threshold[secondCompIndex]))
        //            {
        //                segmentation.join(firstCompIndex, secondCompIndex);
        //                firstCompIndex = segmentation.find(firstCompIndex);
        //                threshold[firstCompIndex] = weight + (c / segmentation.size(firstCompIndex));
        //            }
        //        }
        //    }
        //    sw3.Stop();
        //    Console.WriteLine("Without pointers: {0}", sw3.ElapsedMilliseconds);

        //    int compCount=0;
        //    for (int i = 0; i < segmentation.NumberOfComponents; i++)
        //    {
        //        //Console.WriteLine(segmentation.Components[i].size);
        //        if (segmentation.Components[i].size > 1000)
        //        {
        //            compCount++;
        //        }
                
        //    }
        //    Console.WriteLine("number of segments: {0}", compCount);
        //}

        //private static List<Edge> createComponentEdgesList(float[] depthMap, List<Edge> edgeList)
        //{
        //    for (int i = 1; i < frameHeight - 1; i++)
        //    {
        //        for (int j = 0; j < frameWidth - 1; j++)
        //        {
        //            int currentIndex = i * frameWidth + j;

        //            if (depthMap[currentIndex] != 0)
        //            {
        //                int rightUpIndex = (i - 1) * frameWidth + j + 1;
        //                int rightIndex = i * frameWidth + j + 1;
        //                int rightDownindex = (i + 1) * frameWidth + j + 1;
        //                int downIndex = (i + 1) * frameWidth + j;

        //                float currentPixel = depthMap[currentIndex];
        //                float rightUpPixel = depthMap[rightUpIndex];
        //                float rightPixel = depthMap[rightIndex];
        //                float rightDownPixel = depthMap[rightDownindex];
        //                float downPixel = depthMap[downIndex];

        //                Edge rightUpEdge;
        //                rightUpEdge.firstComponent = currentIndex;
        //                rightUpEdge.secondComponent = rightUpIndex;
        //                rightUpEdge.weight = Math.Abs(currentPixel - rightUpPixel);
        //                edgeList.Add(rightUpEdge);

        //                Edge rightEdge;
        //                rightEdge.firstComponent = currentIndex;
        //                rightEdge.secondComponent = rightIndex;
        //                rightEdge.weight = Math.Abs(currentPixel - rightPixel);
        //                edgeList.Add(rightEdge);

        //                Edge rightDownEdge;
        //                rightDownEdge.firstComponent = currentIndex;
        //                rightDownEdge.secondComponent = rightDownindex;
        //                rightDownEdge.weight = Math.Abs(currentPixel - rightDownPixel);
        //                edgeList.Add(rightDownEdge);

        //                Edge downEdge;
        //                downEdge.firstComponent = currentIndex;
        //                downEdge.secondComponent = downIndex;
        //                downEdge.weight = Math.Abs(currentPixel - downPixel);
        //                edgeList.Add(downEdge);
        //            }
        //        }
        //    }
        //    return edgeList;
        
        //}
        
        //private static void createComponentEdges(float[] depthMap)
        //{
        //    int k = 0;
        //    for (int i = 1; i < frameHeight - 1; i++)
        //    {
        //        for (int j = 0; j < frameWidth - 1; j++)
        //        {
        //            int currentIndex = i * frameWidth + j;
        //            int rightUpIndex = (i - 1) * frameWidth + j + 1;
        //            int rightIndex = i * frameWidth + j + 1;
        //            int rightDownindex = (i + 1) * frameWidth + j + 1;
        //            int downIndex = (i + 1) * frameWidth + j;

        //            float currentPixel = depthMap[currentIndex];
        //            float rightUpPixel = depthMap[rightUpIndex];
        //            float rightPixel = depthMap[rightIndex];
        //            float rightDownPixel = depthMap[rightDownindex];
        //            float downPixel = depthMap[downIndex];

        //            edges[k].firstComponent = currentIndex;
        //            edges[k].secondComponent = rightUpIndex;
        //            edges[k].weight = Math.Abs(currentPixel - rightUpPixel);
        //            k++;
        //            edges[k].firstComponent = currentIndex;
        //            edges[k].secondComponent = rightIndex;
        //            edges[k].weight = Math.Abs(currentPixel - rightPixel);
        //            k++;
        //            edges[k].firstComponent = currentIndex;
        //            edges[k].secondComponent = rightDownindex;
        //            edges[k].weight = Math.Abs(currentPixel - rightDownPixel);
        //            k++;
        //            edges[k].firstComponent = currentIndex;
        //            edges[k].secondComponent = downIndex;
        //            edges[k].weight = Math.Abs(currentPixel - downPixel);
        //            k++;
        //        }
        //    }
        //}



        public static CameraSpacePoint[] MedianFilter3x3(CameraSpacePoint[] pointCloud) 
        {
            float[] kernel3x3 = new float[9];
            int frameWidth = scaledFrameWidth;
            int frameHeight = scaledFrameHeight;
            CameraSpacePoint[] filteredPointCloud = new CameraSpacePoint[frameHeight*frameWidth];
            for (int i = 1; i < frameHeight-1; i++)
            {
                for (int j = 1; j < frameWidth-1; j++)
                {
                    kernel3x3[0] = pointCloud[i * frameWidth + j].Z;
                    kernel3x3[1] = pointCloud[i * frameWidth + j - 1].Z;
                    kernel3x3[2] = pointCloud[(i * frameWidth + j) + 1].Z;
                    kernel3x3[3] = pointCloud[(i - 1) * frameWidth + j].Z;
                    kernel3x3[4] = pointCloud[(i + 1) * frameWidth + j].Z;
                    kernel3x3[5] = pointCloud[(i - 1) * frameWidth + j - 1].Z;
                    kernel3x3[6] = pointCloud[(i - 1) * frameWidth + j + 1].Z;
                    kernel3x3[7] = pointCloud[(i + 1) * frameWidth + j - 1].Z;
                    kernel3x3[8] = pointCloud[(i + 1) * frameWidth + j + 1].Z;

                    Array.Sort(kernel3x3);
                    filteredPointCloud[i * frameWidth + j].Z = kernel3x3[4];
                }
            }
            return filteredPointCloud;
        }

        private static float[] CreateDepthMapFromPointCloud(CameraSpacePoint[] pointCloud)
        {
            float[] depthMap = new float[pointCloud.Length];
            for (int i = 0; i < pointCloud.Length; i++)
            {
                depthMap[i] = pointCloud[i].Z;
            }
            return depthMap;

        }


        public static void GraphBasedSegmentation2(CameraSpacePoint[] pointCloud) 
        {

            // send index instead of value

            Stopwatch stopwatch = new Stopwatch();

            stopwatch.Start();
            int[] labeledFrame = new int[frameHeight*frameWidth];
            float[] depthMap = new float[frameHeight*frameWidth];
            Dictionary<int,float> internalMaxWeights = new Dictionary<int,float>();

            for (int i = 0; i < labeledFrame.Length; i++)
            {
                labeledFrame[i] = i;
            }
            depthMap = CreateDepthMapFromPointCloud(pointCloud);
            stopwatch.Stop();

            Console.WriteLine("Create frames: {0}", stopwatch.ElapsedMilliseconds);

            for (int i = 1; i < frameHeight - 1; i++)
            {
                for (int j = 0; j < frameWidth - 1; j++)
                {
                    //define pointers to edges
                    
                    float currentPixel = depthMap[i*frameWidth + j];
                    float rightUpPixel = depthMap[(i - 1) * frameWidth + j + 1];
                    float rightPixel = depthMap[i * frameWidth + j + 1];
                    float rightDownPixel = depthMap[(i + 1) * frameWidth + j + 1];
                    float downPixel = depthMap[(i + 1) * frameWidth + j];

                    float rightUpEdge = Math.Abs(currentPixel - rightUpPixel);
                    float rightEdge = Math.Abs(currentPixel - rightPixel);
                    float rightDownEdge = Math.Abs(currentPixel - rightDownPixel);
                    float downEdge = Math.Abs(currentPixel - downPixel);



                }
            }
        }


        //public static void GraphBasedSegmentation(CameraSpacePoint[] pointCloud)
        //{


        //    Stopwatch stopwatch = new Stopwatch();
        //    Dictionary<int, Component> segmentation = new Dictionary<int, Component>();
        //    stopwatch.Start();


        //    for (int i = 0; i < pointCloud.Length; i++)
        //    {
        //        CameraSpacePoint pixel = pointCloud[i];
        //        PixelVertex newVertex = new PixelVertex(pixel,i);
        //        vertices[i] = newVertex;
        //        if (pixel.Z != 0)
        //        { 
        //            Component component = new Component(newVertex,i);
        //            segmentation.Add(i,component);
        //        }
        //    }
        //    stopwatch.Stop();
        //    Console.WriteLine("Fillvertices and segmentation: {0}", stopwatch.ElapsedMilliseconds);

        //    stopwatch.Start();
        //    CreatePixelEdges();
        //    pixelEdges.Sort();
        //    stopwatch.Stop();
        //    Console.WriteLine("Create and sort edges: {0}", stopwatch.ElapsedMilliseconds);

        //    stopwatch.Start();
        //    for (int i = 0; i < pixelEdges.Count; i++)
        //    {
        //        PixelEdge edge = pixelEdges[i];
        //        PixelVertex firstVertex = edge.Vertices[0];
        //        PixelVertex secondVertex = edge.Vertices[1];

        //        if (firstVertex.ComponentID != secondVertex.ComponentID)
        //        { 
        //            Component firstComponent;
        //            Component secondComponent;
        //            int firstComponentID = firstVertex.ComponentID;
        //            int secondComponentID = secondVertex.ComponentID;
        //            if(segmentation.TryGetValue(firstComponentID,out firstComponent) && segmentation.TryGetValue(secondComponentID,out secondComponent))
        //            {
        //                if (edge.Weight < firstComponent.InternalMaxWeight && edge.Weight < secondComponent.InternalMaxWeight)
        //                {
        //                    firstComponent.add(secondComponent);
        //                    segmentation.Remove(secondComponentID);
        //                }
        //            }
        //        }
                
        //    }
        //    stopwatch.Stop();
        //    Console.WriteLine("Connect to blobs: {0}", stopwatch.ElapsedMilliseconds);
        //    int t = segmentation.Count;
        //}

        //private static void CalculatePixelDifferences(CameraSpacePoint[] pointCloud) 
        //{
        //    for (int i = 0; i < frameHeight-1; i++)
        //    {
        //        for (int j = 0; j < frameWidth-1; j++)
        //        {
        //            differencesXDirection[i * frameWidth + j] = Math.Abs(pointCloud[i * frameWidth + j].Z - pointCloud[i * frameWidth + j + 1].Z);
        //            differencesYDirection[i * frameHeight + j] = Math.Abs(pointCloud[i * frameWidth + j].Z - pointCloud[(i + 1) * frameWidth + j].Z);
        //        }
        //    }
        //}
        /*
        private static void CreatePixelVerticesWithEdgeDifferences(CameraSpacePoint[] pointCloud)
        {
            for (int i = 1; i < frameHeight-1; i++)
            {
                for (int j = 0; j < frameWidth-1; j++)
                {
                    float[] edges = new float[5];
                    float depthCurrent = pointCloud[i * frameWidth + j].Z;
                    edges[0] = Math.Abs(pointCloud[(i-1)*frameWidth+j].Z - depthCurrent);
                    edges[1] = Math.Abs(pointCloud[(i - 1) * frameWidth + j + 1].Z - depthCurrent);
                    edges[2] = Math.Abs(pointCloud[i * frameWidth + j + 1].Z - depthCurrent);
                    edges[3] = Math.Abs(pointCloud[(i + 1) * frameWidth + j + 1].Z - depthCurrent);
                    edges[4] = Math.Abs(pointCloud[(i + 1) * frameWidth + j].Z - depthCurrent);
                    
                    PixelVertex vertex = new PixelVertex(pointCloud[i*frameWidth + j], edges);
                }
            }
        
        }
        */
        //private static void CreatePixelEdges() 
        //{

        //    // Optimize by not storing cameraPoints in edge
        //    for (int i = 1; i < frameHeight - 1; i++)
        //    {
        //        for (int j = 0; j < frameWidth - 1; j++)
        //        {
        //            PixelVertex currentPixel = vertices[i * frameWidth + j];
        //            float currentPixelDepth = currentPixel.CameraSpacePoint.Z;

        //            if (currentPixelDepth != 0)
        //            {
        //                PixelVertex upRightPixel = vertices[(i - 1) * frameWidth + j + 1];
        //                PixelVertex rightPixel = vertices[i * frameWidth + j + 1];
        //                PixelVertex downRightPixel = vertices[(i + 1) * frameWidth + j + 1];
        //                PixelVertex downPixel = vertices[(i + 1) * frameWidth + j];

        //                PixelEdge edgeRightUp = new PixelEdge(new PixelVertex[2] { currentPixel, upRightPixel }, Math.Abs(upRightPixel.CameraSpacePoint.Z - currentPixelDepth));
        //                pixelEdges.Add(edgeRightUp);
        //                PixelEdge edgeRight = new PixelEdge(new PixelVertex[2] { currentPixel, rightPixel }, Math.Abs(rightPixel.CameraSpacePoint.Z - currentPixelDepth));
        //                pixelEdges.Add(edgeRight);
        //                PixelEdge edgeRightDown = new PixelEdge(new PixelVertex[2] { currentPixel, downRightPixel }, Math.Abs(downRightPixel.CameraSpacePoint.Z - currentPixelDepth));
        //                pixelEdges.Add(edgeRightDown);
        //                PixelEdge edgeDown = new PixelEdge(new PixelVertex[2] { currentPixel, downPixel }, Math.Abs(downPixel.CameraSpacePoint.Z - currentPixelDepth));
        //                pixelEdges.Add(edgeDown);
        //            }
        //        }
        //    }

        //}



        //public static unsafe void SegmentGraphBackup(float c, CameraSpacePoint[] pointCloud)
        //{
        //    float[] depthMap = CreateDepthMapFromPointCloud(pointCloud);

        //    Stopwatch sw = new Stopwatch();
        //    sw.Start();
        //    List<edge> edgeList = new List<edge>();
        //    edgeList = createComponentEdgesList(depthMap, edgeList);
        //    edgeList.Sort();
        //    sw.Stop();
        //    Console.WriteLine("List: {0}", sw.ElapsedMilliseconds);
        //    int numEdgesList = edgeList.Count;


        //    //Stopwatch sw2 = new Stopwatch();
        //    //sw2.Start();
        //    //int numEdges = edges.Length;
        //    //createComponentEdges(depthMap);
        //    //Array.Sort(edges);
        //    //sw2.Stop();
        //    //Console.WriteLine("Non-generic: {0}", sw2.ElapsedMilliseconds);


        //    SegmentationSet segmentation = new SegmentationSet(numVertices);

        //    fixed (float* pThreshold = threshold)
        //    {
        //        for (int i = 0; i < numVertices; i++)
        //        {
        //            pThreshold[i] = c / 1;
        //        }
        //    }

        //    Stopwatch sw3 = new Stopwatch();
        //    sw3.Start();
        //    for (int i = 0; i < numEdgesList; i++)
        //    {

        //        edge currentEdge = edgeList.ElementAt(i);
        //        int firstComp = currentEdge.firstComponent;
        //        int secondComp = currentEdge.secondComponent;
        //        float weight = currentEdge.weight;

        //        int firstCompIndex = segmentation.find(firstComp);
        //        int secondCompIndex = segmentation.find(secondComp);

        //        if (firstCompIndex != secondCompIndex)
        //        {
        //            if ((weight <= threshold[firstCompIndex]) && (weight <= threshold[secondCompIndex]))
        //            {
        //                segmentation.join(firstCompIndex, secondCompIndex);
        //                firstCompIndex = segmentation.find(firstCompIndex);
        //                threshold[firstCompIndex] = weight + (c / segmentation.size(firstCompIndex));
        //            }
        //        }
        //    }
        //    sw3.Stop();
        //    Console.WriteLine("Without pointers: {0}", sw3.ElapsedMilliseconds);

        //    //Stopwatch sw4 = new Stopwatch();
        //    //sw.Start();
        //    //for (int i = 0; i < numEdges; i++)
        //    //{
        //    //    fixed (int* pFirstComp = &edges[i].firstComponent, pSecondComp = &edges[i].secondComponent)
        //    //    {
        //    //        fixed(float *pWeight = &edges[i].weight)
        //    //        {
        //    //            int firstCompIndex = segmentation.find(*pFirstComp);
        //    //            int secondCompIndex = segmentation.find(*pSecondComp);

        //    //            if (firstCompIndex != secondCompIndex)
        //    //            {
        //    //                if ((*pWeight<=threshold[firstCompIndex]) && (*pWeight<=threshold[secondCompIndex]))
        //    //                {
        //    //                    segmentation.join(firstCompIndex,secondCompIndex);
        //    //                    firstCompIndex = segmentation.find(firstCompIndex);
        //    //                    threshold[firstCompIndex] = *pWeight + (c / segmentation.size(firstCompIndex));
        //    //                }
        //    //            }
        //    //        }
        //    //    }
        //    //}
        //    //sw4.Stop();
        //    //Console.WriteLine("with Pointers: {0}", sw.ElapsedMilliseconds);
        //    int compCount = 0;
        //    for (int i = 0; i < segmentation.NumberOfComponents; i++)
        //    {
        //        //Console.WriteLine(segmentation.Components[i].size);
        //        if (segmentation.Components[i].size > 1000)
        //        {
        //            compCount++;
        //        }

        //    }
        //    Console.WriteLine("number of segments: {0}", compCount);
        //}
    }


}
