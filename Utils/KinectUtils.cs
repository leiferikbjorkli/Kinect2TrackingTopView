//
// Written by Leif Erik Bjoerkli
//


using System;
using Microsoft.Kinect;


namespace InteractionDetection
{
    public static class KinectUtils
    {

        public static CameraSpacePoint[] DepthFilterBackgroundSubtraction(CameraSpacePoint[] cameraSpacePoints)
        {
            CameraSpacePoint[] pointCloud = new CameraSpacePoint[cameraSpacePoints.Length];

            float maxDepth = GlobVar.MaxDepthMeter;
            float minDepth = GlobVar.MinDepthMeter;
            float maxXWidth = GlobVar.MaxXWidth;
            float maxYWidth = GlobVar.MaxYWidth;

            for (int i = 0; i < cameraSpacePoints.Length; i++)
            {
                CameraSpacePoint cameraSpacePoint = cameraSpacePoints[i];

                if (cameraSpacePoint.Z < minDepth && cameraSpacePoint.Z > maxDepth && Math.Abs(cameraSpacePoint.X) > maxXWidth && Math.Abs(cameraSpacePoint.Y) > maxYWidth)
                {
                    cameraSpacePoint.Z = maxDepth;
                }

                pointCloud[i] = cameraSpacePoint;

            }
            return pointCloud;
        }

        public static CameraSpacePoint[] ScaleFrame(CameraSpacePoint[] cameraSpacePoints)
        { 
            CameraSpacePoint[] newFrame = new CameraSpacePoint[GlobVar.ScaledFrameLength];
            int k = 0;

            var frameHeight = GlobVar.FrameHeight;
            var frameWidth = GlobVar.FrameWidth;
            var maxDepth = GlobVar.MaxDepthMeter;

            for (int i = 0; i < frameHeight - 1; i += 2)
            {
                for (int j = 0; j < frameWidth - 1; j += 2)
                {
                    float sumX = 0;
                    float sumY = 0;
                    float sumZ = 0;
                    int validPixels = 0;

                    CameraSpacePoint upLeft = cameraSpacePoints[(i * frameWidth) + j];
                    float depthCurrent = upLeft.Z;
                    if (!float.IsInfinity(depthCurrent))
                    {
                        sumX += upLeft.X;
                        sumY += upLeft.Y;
                        sumZ += depthCurrent;
                        validPixels++;
                    }
                    CameraSpacePoint right = cameraSpacePoints[(i * frameWidth) + j + 1];
                    float depthRight = right.Z;
                    if (!float.IsInfinity(depthRight))
                    {
                        sumX += right.X;
                        sumY += right.Y;
                        sumZ += depthRight;
                        validPixels++;
                    }
                    CameraSpacePoint down = cameraSpacePoints[(i * frameWidth + 1) + j];
                    float depthDown = down.Z;
                    if (!float.IsInfinity(depthDown))
                    {
                        sumX += down.X;
                        sumY += down.Y;
                        sumZ += depthDown;
                        validPixels++;
                    }
                    CameraSpacePoint rightDown = cameraSpacePoints[(i * frameWidth + 1) + j + 1];
                    float depthRightDown = rightDown.Z;
                    if (!float.IsInfinity(depthRightDown))
                    {
                        sumX += rightDown.X;
                        sumY += rightDown.Y;
                        sumZ += depthRightDown;
                        validPixels++;
                    }
                    if (validPixels == 0)
                    {
                        newFrame[k] = new CameraSpacePoint()
                        {
                            X = float.PositiveInfinity,
                            Y = float.PositiveInfinity,
                            Z = maxDepth
                        };
                    }
                    else
                    {
                        newFrame[k].X = sumX / validPixels;
                        newFrame[k].Y = sumY / validPixels;
                        newFrame[k].Z = sumZ / validPixels;
                    }
                    k++;
                }
            }
            return newFrame;
        }

        public static CameraSpacePoint[] CreatePointCloud(CameraSpacePoint[] cameraSpacePoints)
        {
            // Faster to use constants than reading framedescription Faster to
            // declare variable outside loop and reuse memory location Rarely
            // used parts of method should be put into seperate method Faster to
            // calculate maths in a long line than temp storage

            // Optimize by initializing simply types instead of accessing fields
            CameraSpacePoint[] pointCloud = new CameraSpacePoint[cameraSpacePoints.Length];
            for (int i = 0; i < pointCloud.Length; i++)
            {
                pointCloud[i].Z = GlobVar.MaxDepthMeter;
            }

            int frameWidth = GlobVar.ScaledFrameWidth;
            int frameHeight = GlobVar.ScaledFrameHeight;

            foreach (var cameraSpacePoint in cameraSpacePoints)
            {
                if ((int)(cameraSpacePoint.Z * 1000) > GlobVar.MinDepth && (int)(cameraSpacePoint.Z * 1000) < GlobVar.MaxDepth && Math.Abs(cameraSpacePoint.X) < GlobVar.MaxXWidth && Math.Abs(cameraSpacePoint.Y) < GlobVar.MaxYWidth)
                {
                    int xPixelCoordinateOfPoint = (int)Math.Round(((cameraSpacePoint.X * 1000) / GlobVar.HalfMaxHorizontalWidth) * (frameWidth / 2) + (frameWidth / 2));
                    int yPixelCoordinateOfPoint = (int)Math.Round(((cameraSpacePoint.Y * 1000) / GlobVar.HalfMaxVerticalHeight) * (frameHeight / 2) + (frameHeight / 2));

                    if (xPixelCoordinateOfPoint >= frameWidth)
                    {
                        xPixelCoordinateOfPoint = frameWidth-1;
                    }
                    if (yPixelCoordinateOfPoint > frameHeight)
                    {
                        yPixelCoordinateOfPoint = frameHeight;
                    }

                    if (xPixelCoordinateOfPoint < 1)
                    {
                        xPixelCoordinateOfPoint = 0;
                    }
                    if (yPixelCoordinateOfPoint < 1)
                    {
                        yPixelCoordinateOfPoint = 1;
                    }

                    //Flip to fit kinect studio
                    int pointIndex = ((frameHeight - yPixelCoordinateOfPoint) * frameWidth) + xPixelCoordinateOfPoint;
                    if (cameraSpacePoint.Z < pointCloud[pointIndex].Z || pointCloud[pointIndex].Z == GlobVar.MaxDepthMeter)
                    {
                        pointCloud[pointIndex] = cameraSpacePoint;
                    }
                }
            }
            return pointCloud;
        }

        public static byte[] CalculateIntensityFromCameraSpacePoints(CameraSpacePoint[] cameraSpacePoint)
        {
            var intensityMap = new byte[cameraSpacePoint.Length];

            for (int i = 0; i < cameraSpacePoint.Length; i++)
            {
                float depthM = cameraSpacePoint[i].Z;
                if (depthM != 0)
                {
                    int depthMM = (int)Math.Round(depthM * 1000);
                    int currentMax = depthMM - GlobVar.MinDepth;
                    int currentDepthRange = GlobVar.MaxDepth - GlobVar.MinDepth;

                    if (depthMM < GlobVar.MaxDepth && depthMM > GlobVar.MinDepth)
                    {
                        intensityMap[i] = (byte)(255 - (255 * currentMax / currentDepthRange));
                    }
                    else
                    {
                        intensityMap[i] = (byte)0;
                    }
                   
                }
                else 
                {
                    intensityMap[i] = (byte)0;
                }
            }
            return intensityMap;
        }

        public static Point GetFramePointFromCameraSpace(CameraSpacePoint p)
        {
            var depthSpacePoint = GlobVar.CoordinateMapper.MapCameraPointToDepthSpace(p);

            return new Point((int)Math.Round(depthSpacePoint.X) / 2, (int)Math.Round(depthSpacePoint.Y) / 2);
        }
        public static int GetFrameIndexFromCameraSpace(CameraSpacePoint p)
        {
            var depthSpacePoint = GlobVar.CoordinateMapper.MapCameraPointToDepthSpace(p);


            return GlobUtils.GetIndex(new Point((int)Math.Round(depthSpacePoint.X) / 2, (int)Math.Round(depthSpacePoint.Y) / 2));
        }
    }


}
