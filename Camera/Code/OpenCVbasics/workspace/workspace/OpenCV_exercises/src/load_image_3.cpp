#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

void loadImage3()
{
    // load images
    vector<cv::Mat> imgList; //A dynamic list of type vector<cv::Mat>.
    for (int i = 5; i <= 9; i++)
    {
        // create file name
        ostringstream imgNumber;                   // #include <sstream>
        imgNumber << setfill('0') << setw(4) << i; // #include <iomanip>
        string filename = "../images/img" + imgNumber.str() + ".jpg";

        // load image and store it into a vector
        cv::Mat img;
        img = cv::imread(filename);
        imgList.push_back(img); // store pointer to current image in list
    }

    // display images from the vector
    string windowName = "First steps in OpenCV";
    cv::namedWindow(windowName, 1); // create window
    // Note the auto keyword is simply asking the compiler to deduce the type of the variable from the initialization, which is much more convenient than writing (vector<cv::Mat>::iterator it) instead
    for (auto it = imgList.begin(); it != imgList.end(); ++it)
    {

        // STUDENT TASK : Prevent image 7 from being displayed
        if (it == imgList.begin() + 2)
        {
            continue;
        }
        // display image
        cv::imshow(windowName, *it);
        cv::waitKey(0); // wait for keyboard input before continuing
    }
}

int main()
{
    loadImage3();
    return 0;
}