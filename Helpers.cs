using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.IO;
using System.Drawing;

namespace Microsoft.Samples.Kinect.DepthBasics
{
    public static class Helpers
    {


        public static Bitmap writeableBitmapToBitmap(WriteableBitmap writeBitmap)
        {
            Bitmap bmp;
            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder encoder = new BmpBitmapEncoder();
                encoder.Frames.Add(BitmapFrame.Create(writeBitmap));
                encoder.Save(outStream);
                bmp = new Bitmap(outStream);
            }
            return bmp;

        }

        public static Bitmap byteArrayToBitmap(byte[] pixelArray) {

            MemoryStream ms = new MemoryStream(pixelArray);
            Image image = Image.FromStream(ms);
            Bitmap bitmap = new Bitmap(image);
            return bitmap;
        }

        public static byte CalculateIntensityFromDistance(int depth)
            {
            // This will map a distance value to a 0 - 255 range
            // for the purposes of applying the resulting value
            // to RGB pixels.
            int minDepth = 1500;
            int maxDepth = 2230;
            int intervalHeight = (maxDepth-minDepth);

            int currentDepthRange = maxDepth - minDepth;

            int currentMax = depth - minDepth;

            //if (depth < maxDepth && depth > minDepth)
            //{

            //    if (depth > (minDepth + intervalHeight))
            //        if ((depth % intervalHeight) == 0)
            //            return (byte)255;
            //        else
            //            return (byte)(255 * (depth % intervalHeight) / intervalHeight);
            //    else if (depth < (minDepth + intervalHeight))
            //        if ((depth % intervalHeight) == 0)
            //            return (byte)0;
            //        else
            //            return (byte)(255 - Math.Ceiling((float)(255 * (depth % intervalHeight) / intervalHeight)));
            //    else
            //        return 100;
            //}else
            //    return (byte)0;
            if (depth < maxDepth && depth > minDepth)
                return (byte)(255 - (255 * currentMax / currentDepthRange));
            else
                return (byte)0;



        }


        /// <summary>
        /// Converts a <see cref="System.Drawing.Bitmap"/> into a WPF <see cref="BitmapSource"/>.
        /// </summary>
        /// <remarks>Uses GDI to do the conversion. Hence the call to the marshalled DeleteObject.
        /// </remarks>
        /// <param name="source">The source bitmap.</param>
        /// <returns>A BitmapSource</returns>
        public static BitmapSource ToBitmapSource(this System.Drawing.Bitmap source)
        {
            BitmapSource bitSrc = null;

            var hBitmap = source.GetHbitmap();

            try
            {
                bitSrc = System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(
                    hBitmap,
                    IntPtr.Zero,
                    Int32Rect.Empty,
                    BitmapSizeOptions.FromEmptyOptions());

               

            }
            catch (Win32Exception)
            {
                bitSrc = null;
            }
            finally
            {
                NativeMethods.DeleteObject(hBitmap);
            }

            //var grayBitmap = new FormatConvertedBitmap();
            //grayBitmap.BeginInit();
            //grayBitmap.Source = bitSrc;
            //grayBitmap.DestinationFormat = PixelFormats.;
            //grayBitmap.EndInit();


            return bitSrc;

        }

        

        /// <summary>
        /// FxCop requires all Marshalled functions to be in a class called NativeMethods.
        /// </summary>
        internal static class NativeMethods
        {
            [DllImport("gdi32.dll")]
            [return: MarshalAs(UnmanagedType.Bool)]
            internal static extern bool DeleteObject(IntPtr hObject);
        }
    }
}
