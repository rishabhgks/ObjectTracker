// Import all the dependent packages
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <boost/range/adaptor/sliced.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

const int N = 28;
using boost::adaptors::sliced;

int main(int argc, char **argv)
{
// List of tracker types in OpenCV 3.2
    string trackerTypes[7] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "CSRT"};
    // vector <string> trackerTypes(types, std::end(types));

// Create a tracker
    string trackerType = trackerTypes[6];

    Ptr<Tracker> tracker;
    Ptr<Tracker> tracker2;

//#if (CV_MINOR_VERSION < 3)
//    {
//        tracker = Tracker::create(trackerType);
//    }
//#else
    {
// Initialize the tracker object based on the tracker type selected
        if (trackerType == "BOOSTING")
            tracker = TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker = TrackerMIL::create();
        if (trackerType == "KCF")
            tracker = TrackerKCF::create();
        if (trackerType == "TLD")
            tracker = TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker = TrackerMedianFlow::create();
        if (trackerType == "GOTURN")
            tracker = TrackerGOTURN::create();
        if (trackerType == "CSRT")
            tracker = TrackerCSRT::create();
    }

    {
        if (trackerType == "BOOSTING")
            tracker2 = TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker2 = TrackerMIL::create();
        if (trackerType == "KCF")
            tracker2 = TrackerKCF::create();
        if (trackerType == "TLD")
            tracker2 = TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker2 = TrackerMedianFlow::create();
        if (trackerType == "GOTURN")
            tracker2 = TrackerGOTURN::create();
        if (trackerType == "CSRT")
            tracker2 = TrackerCSRT::create();
    }
//#endif
    // Read video
    VideoCapture video("/home/rishabh/Downloads/Hovering.mp4");

    // Exit if video is not opened
    if(!video.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;

    }

    // Read first frame
    Mat frame;
    bool ok = video.read(frame);
//    bool outOk = out_video.

    VideoWriter out_video("/home/rishabh/Downloads/Hovering_edit_nbb.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'), 30, Size(frame.size[1], frame.size[0]));

    // Define initial boundibg box
    Rect2d bbox(287, 23, 86, 320);
    Rect2d bbox2(287, 23, 86, 320);

    // Uncomment the line below to select a different bounding box
    bbox = selectROI(frame, false);
    bbox2 = selectROI(frame, false);

    vector<Point> uav1_center_points;
    vector<Point> uav2_center_points;

    // Display bounding box.
    rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
    rectangle(frame, bbox2, Scalar( 255, 0, 0 ), 2, 1 );
    imshow("Tracking", frame);

    tracker->init(frame, bbox);
    tracker2->init(frame, bbox2);


    while(video.read(frame))
    {

        // Start timer
        double timer = (double)getTickCount();

        // Update the tracking result
        bool ok = tracker->update(frame, bbox);
        bool ok2 = tracker2->update(frame, bbox2);

        // Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double)getTickCount() - timer);


        if (ok)
        {
            uav1_center_points.push_back(Point(bbox.x + (bbox.width/2), bbox.y + (bbox.height/2)));
            // Tracking success : Draw the tracked object
//            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
            for(int i = 0; i < uav1_center_points.size(); i+=15)
                circle(frame, uav1_center_points[i], 1, Scalar( 255, 0, 0 ), 2, 1);
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }

        if (ok2)
        {
            uav2_center_points.push_back(Point(bbox2.x + (bbox2.width/2), bbox2.y + (bbox2.height/2)));
            // Tracking success : Draw the tracked object
//            rectangle(frame, bbox2, Scalar( 255, 0, 0 ), 2, 1 );
            for(int i = 0; i < uav2_center_points.size(); i+=15)
                circle(frame, uav2_center_points[i], 1, Scalar( 255, 0, 0 ), 2, 1);
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }


        // Display tracker type on frame
//        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

        // Display FPS on frame
//        putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        // Display frame.
//        imshow("Tracking", frame);
        out_video.write(frame);
        // Exit if ESC pressed.
        int k = waitKey(1);
        if(k == 27)
        {
            break;
        }

    }

    out_video.release();

}