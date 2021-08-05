// Import all the dependent packages
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv4/opencv2/core/ocl.hpp>
#include <fstream>
#include <string>
#include <ctime>

using namespace cv;
using namespace std;
Mat frame;
int thresh = 150;
const int max_thresh = 255;
const char* source_window = "Tracking";
int dil_iter = 6;
int ero_iter = 6;
RNG rng(12345);
int min_width_check = 70;
int max_width_check = 300;
int aspect_ratio = 3.5;
Ptr<Tracker> tracker;
Rect bbox, bbox_obj;
bool initializeTracker = false;
bool drawTracker = false;
bool detectedContour = false;
VideoCapture video("/home/rishabh/Downloads/2020_09_24_16_29_02.mp4");
// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

void thresh_callback(int, void* );
void object_tracker_callback(int, void* );
bool do_overlap(Rect2d, Rect2d);

int main(int argc, char **argv)
{
// List of tracker types in OpenCV 4.5
    string trackerTypes[3] = {"KCF", "GOTURN", "CSRT"};
    // vector <string> trackerTypes(types, std::end(types));

// Create a tracker -> CSRT
    string trackerType = trackerTypes[2];

//#if (CV_MINOR_VERSION < 3)
//    {
//        tracker = Tracker::create(trackerType);
//    }
//#else
    {
// Initialize the tracker object based on the tracker type selected
        if (trackerType == "KCF")
            tracker = TrackerKCF::create();
        if (trackerType == "GOTURN")
            tracker = TrackerGOTURN::create();
        if (trackerType == "CSRT")
            tracker = TrackerCSRT::create();
    }
//#endif

// Read video
// Exit if video is not opened
    if(!video.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;

    }

    namedWindow( source_window );

// Read first frame
    Mat drone_image;
    bool ok = video.read(frame);
    for(int i {0}; i < 50; i++)
        video.read(frame);


    int counter = 0;
    int detectCounter = 0;

// Declare the output CSV file name which will store the tracking information
    std::ofstream myFile("drone_center_csrt.csv");

// Load the column names of the CSV file - Center X and Center Y value of the drone bounding box detected
    myFile << "Center_x,Center_y" << "\n";

    clock_t start;

    while(video.read(frame)) {
// Note the start time
        start = std::clock();
// Write every 100th frame
        if(counter % 100 == 0)
            imwrite("/home/rishabh/genesys_ws/ObjectTracker/original_uav/drone" + to_string(counter) + ".jpg", frame);

// Resize the frame to reduce computation time and just enough to not harm the pixel features
        resize(frame, frame, Size(frame.size[1]*3/4, frame.size[0]*3/4), cv::INTER_AREA);
// Call the Contour function to detect the UAV in current frame
        thresh_callback(0, 0);
// Call the Object tracker function that uses CSRT tracker to detect the UAV in current frame
        object_tracker_callback(0, 0);
//        cout << ( std::clock() - start ) / (double) CLOCKS_PER_SEC << endl;
// View the current frame with the detected bounding boxes of the UAV
        imshow(source_window, frame);
// Write every 100th tracked frame
        if(counter % 100 == 0)
            imwrite("/home/rishabh/genesys_ws/ObjectTracker/tracked_uav/drone" + to_string(counter) + ".jpg", frame);


        counter++;
// If a Bounding Box has been successfully detected through contouring then load the location in the CSV file for that frame
        if(detectedContour)
        {
            myFile << bbox_obj.x + (bbox_obj.width/2) << "," << bbox_obj.y + (bbox_obj.height/2) << "\n";
            detectCounter++;
        }
// Else load the Bounding box location from the OBject Tracker
        else {
            myFile << bbox.x + (bbox.width/2) << "," << bbox.y + (bbox.height/2) << "\n";
        }

// Exit if ESC pressed.
        int k = waitKey(1);
        if(k == 27)
        {
            break;
        }

    }

    myFile.close();
    cout << counter << " " << detectCounter << endl;

}

// Function to detect UAV in current frame using Contour properties of Image Processing
void thresh_callback(int, void* )
{
    Mat hsv_img, gray;
    int contourCount = 0;
// Convert the image to Grayscale
    cvtColor(frame, gray, COLOR_BGR2GRAY);
// Perform Blurring on the current frame using a small kernel to average out pixel values
    GaussianBlur(gray, gray, Size(3,3), 0, 0);
// Use a Canny Edge Detector to perform thresholding over the grayscale frame
    Canny( gray, gray, thresh, thresh*2 );
    Mat element = getStructuringElement( MORPH_RECT,
                                         Size( 2*1 + 1, 2*1+1 ),
                                         Point( 1, 1 ) );
// Perform Dilation over the current thresholded frame for a set number of times
    dilate(gray, gray, element, Point(1,1), dil_iter);
// Perform Erosion over the current thresholded frame for a set number of times
    erode(gray, gray, element, Point(1,1), ero_iter);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
// Find all the contours in the current frame
    findContours( gray, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );
    vector<vector<Point> >hull( contours.size() );
//    cout << contours.size() << endl;
    for( size_t i = 0; i < contours.size(); i++ )
    {
// Calculate the Convex Hull for all the contours detected and store them
        convexHull( contours[i], hull[i] );
    }
    Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );
    Rect2d bbox_con;
// Loop over all the contours
    for( size_t i = 0; i< contours.size(); i++ )
    {
// Obtain the bounding box around current contour
        bbox_con = boundingRect(contours[i]);
// If the bounding box matches the aspect ratio criteria and the hull matches the number of sides criteria and the contour area is within an acceptable range
// Then we can confirm that we have found the UAV in the frame
        if ((bbox_con.width / bbox_con.height) < 2.5 || (bbox_con.width / bbox_con.height) > aspect_ratio ||
            bbox_con.width < min_width_check || bbox_con.width > max_width_check || hull[i].size() < 9 ||
            contourArea(contours[i])/(bbox_con.width*bbox_con.height) < 0.4)
            continue;
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
// Draw a rectangle around the detected UAV in the frame
        rectangle(frame, bbox_con, Scalar( 0, 255, 0 ), 1, 1 );
//        putText(frame, "BBW : " + SSTR(bbox_con.width), Point(bbox_con.x,bbox_con.y), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 1);
//        putText(frame, "Area : " + SSTR(bbox_con.width*bbox_con.height), Point(bbox_con.x - bbox_con.width,bbox_con.y + bbox_con.height), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 1);
// Load the detected UAV bounding box in a variable
        bbox_obj = bbox_con;
//        cout << contourCount << endl;
        contourCount++;
    }

// If the CSRT Object tracker has not been initialized and a UAV has been detected then Initialize the tracker
    if(contourCount == 1 && !initializeTracker)
    {
        tracker->init(frame, bbox_obj);
        initializeTracker = true;
    }
// If the tracker is initialized then update that Contouring has successfully detected a UAV
    else if (contourCount == 1 && initializeTracker) {
        detectedContour = true;
    } else
        detectedContour = false;

// Enable drawing the bounding box if this is the first detection after a few frames of no detection
    if(contourCount == 0)
        drawTracker = true;
    else
        drawTracker = false;

    // Display frame.
//    imshow(source_window, drawing);
}

// Function that runs the CSRT object tracker to continously detect the UAV across all frames
void object_tracker_callback(int, void* ) {
// Display bounding box.
    if (initializeTracker)
    {
// Start timer
        double timer = (double) getTickCount();

// Reinitialize the OpenCV Object Tracker if contouring has positively detected a UAV and it does not match the Object tracker bounding box location
        if(do_overlap(bbox, bbox_obj) && detectedContour)
        {
            tracker.release();
            Ptr<Tracker> trackerNew = TrackerCSRT::create();

            tracker = trackerNew;
            tracker->init( frame, bbox_obj );
            putText(frame, "Updated Tracking bounding box", Point(100, 20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50), 2);
        }

        bool ok = tracker->update(frame, bbox);

// Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double) getTickCount() - timer);

        if (ok && drawTracker) {
// Tracking success : Draw the tracked object
            rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
        } else {
// Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75,
                    Scalar(0, 0, 255), 2);
        }

// Display FPS on frame
        putText(frame, "FPS : " + SSTR(int(fps)), Point(100, 50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50, 170, 50),
                2);
    }
}

// Function that compares the location of both the bounding boxes that are detecting the UAV
bool do_overlap(Rect2d box1, Rect2d box2)
{
    // If one rectangle is on left side of other
    if ((box1.x <= box2.x && box2.x <= (box1.x + box1.width) && (box2.x + box2.width) > (box1.x + box1.width)) ||
        (box2.x <= box1.x && box1.x <= (box2.x + box2.width)) && (box1.x + box1.width) > (box2.x + box2.width))
        return true;

    // If one rectangle is above other
    if ((box1.y <= box2.y && box2.y <= (box1.y + box1.height) && (box2.y + box2.height) > (box1.y + box1.height)) ||
        (box2.y <= box1.y && box1.y <= (box2.x + box2.height) && (box1.y + box1.height) > (box2.y + box2.height)))
        return true;

    return false;
}