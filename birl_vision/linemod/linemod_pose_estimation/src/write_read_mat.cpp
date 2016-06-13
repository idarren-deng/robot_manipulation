#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;

int main(int argc,char** argv)
{
    Mat R1=(Mat_<int>(3,3)<<0,1,2,3,4,5,6,7,8);
    Mat R2=(Mat_<int>(3,3)<<8,9,10,3,4,5,6,7,8);
    Mat R3=(Mat_<int>(3,3)<<8,9,10,3,4,5,6,7,8);
    Mat T1=(Mat_<float>(1,3)<<1.2,3.2,4.2);
    Mat T2=(Mat_<float>(1,3)<<2.2,3.2,4.2);
    Mat T3=(Mat_<float>(1,3)<<2.2,3.2,4.2);
    std::vector<Mat> Rs,Ts;
    Rs.push_back (R1);
    Rs.push_back (R2);
    Rs.push_back (R3);
    Ts.push_back (T1);
    Ts.push_back (T2);
    Ts.push_back (T3);

    FileStorage fs("test.yml",FileStorage::WRITE);
    for(int i=0;i<3;++i)
    {
        std::stringstream ss;
        std::string s,a;
        ss<<i;
        s=ss.str ();
        a="Template ";
        a+=s;
        fs<<a<<"{";
        fs<<"ID"<<i;
        fs<<"R"<<Rs[i];
        fs<<"T"<<Ts[i];
        fs<<"}";
    }
    fs.release ();

    FileStorage fs2("test.yml",FileStorage::READ);
    std::vector<cv::Mat> RR,TT;
    for(int i=0;;++i)
    {
        std::stringstream ss2;
        std::string s2;
        s2="Template ";
        ss2<<i;
        s2+=ss2.str ();
        FileNode template2 = fs2[s2];
        if(!template2.empty ())
         {
            int ID;
            Mat R_tmp,T_tmp;
            template2["R"]>>R_tmp;
            RR.push_back (R_tmp);
            template2["T"]>>T_tmp;
            TT.push_back (T_tmp);
            template2["ID"]>>ID;
            std::cout<<ID<<std::endl;
            std::cout<<R_tmp<<std::endl;
            std::cout<<T_tmp<<std::endl;
         }
        else
            break;
    }

    fs2.release ();
}
