using System;

using Emgu;
using Emgu.CV;
using Emgu.CV.Structure;

namespace AutoRobotGUIOpenCV
{
    public class Class1
    {

        public static void Main()
        {
            Mat img1 = CvInvoke.Imread("C:/Users/Nicolas/Documents/car-png-top-view-png-white-top-car-png-image-34867-587.png");
            CvInvoke.NamedWindow("AutoRobotSim");
            Mat imgresize = new Mat();
            CvInvoke.Resize(img1, imgresize, new System.Drawing.Size(100, 100));
            CvInvoke.Imshow("AutoRobotSim", imgresize);
            
            while (true)
                CvInvoke.WaitKey(100);
        }

        
    }
}
