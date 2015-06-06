using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace InteractionDetection
{
    class MathUtils
    {

        // <summary>
        // Fits a line to a collection of (x,y) points.
        // </summary>
        // <param name="xVals">The x-axis values.</param>
        // <param name="yVals">The y-axis values.</param>
        // <param name="inclusiveStart">The inclusive inclusiveStart index.</param>
        // <param name="exclusiveEnd">The exclusive exclusiveEnd index.</param>
        // <param name="rsquared">The r^2 value of the line.</param>
        // <param name="yintercept">The y-intercept value of the line (i.e. y = ax + b, yintercept is b).</param>
        // <param name="slope">The slop of the line (i.e. y = ax + b, slope is a).</param>
        public static void LinearRegression(CameraSpacePoint[] pointCloud)
        {
            float sumOfX = 0;
            float sumOfY = 0;
            float sumOfXSq = 0;
            float sumOfYSq = 0;
            float ssX = 0;
            //float ssY = 0;
            float sumCodeviates = 0;
            float sCo = 0;
            int count = pointCloud.Length;

            for (int ctr = 0; ctr < count; ctr++)
            {
                float x = pointCloud[ctr].X;
                float y = pointCloud[ctr].Y;
                sumCodeviates += x * y;
                sumOfX += x;
                sumOfY += y;
                sumOfXSq += x * x;
                sumOfYSq += y * y;
            }
            ssX = sumOfXSq - ((sumOfX * sumOfX) / count);
            //ssY = sumOfYSq - ((sumOfY * sumOfY) / count);
            float RNumerator = (count * sumCodeviates) - (sumOfX * sumOfY);
            float RDenom = (count * sumOfXSq - (sumOfX * sumOfX)) * (count * sumOfYSq - (sumOfY * sumOfY));
            sCo = sumCodeviates - ((sumOfX * sumOfY) / count);

            float meanX = sumOfX / count;
            float meanY = sumOfY / count;

            //Graphics.DrawPointNoPointcloud(meanX,meanY);

            float dblR = RNumerator / (float)Math.Sqrt(RDenom);

            float rsquared = dblR * dblR;
            float yintercept = meanY - ((sCo / ssX) * meanX);
            float slope = sCo / ssX;

            Graphics.DrawLineNoPointcloud(slope,yintercept,rsquared);



        }
    }
}
