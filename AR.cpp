#include <iostream>
#include <fstream>
#include <string>
#include <cstddef>
#include <map>
#include <algorithm>
#include "loadpng.h"

using namespace std;

void readPNG(const string, vector<unsigned char>&, unsigned&, unsigned&);
void savePNG(const string filename, vector<unsigned char>& image, unsigned width, unsigned height);
void labelByDepth(vector<unsigned char>&, vector<unsigned char>&, unsigned, unsigned);
void findNeighbor(vector<unsigned char>& label, vector<unsigned char>& dImage, vector<size_t>& pickList, unsigned width, unsigned height
                  , unsigned char startD,size_t pickNum,unsigned char labelNum);
bool checkNeverUsed(size_t newNum, vector<size_t>& pickList);

int main(int argc, char ** argv)
{
  if(argc < 3){
    cout<<"INFO: ./AR <png_directory> <result_dirctory>"<<endl<<"  EX: ./AR desk_1 desk_1_result"<<endl;
    return false;
  }
  const string file_prefix = argv[1];
  const string result_prefix = argv[2];
  const string RESULT_FILENAME = result_prefix + '/' +result_prefix;
  const string FILENAME = file_prefix + '/' +file_prefix;
  unsigned width1,height1,width2,height2;
  for(int i =1;i<=98;i++){
    string RGBfilename =FILENAME+"_"+to_string(i)+".png";
    string Dfilename = FILENAME+"_"+to_string(i)+"_depth.png";
    vector<unsigned char> rgbImage;
    vector<unsigned char> tempImage;
    cout<<RGBfilename<<endl<<Dfilename<<endl;

    readPNG(RGBfilename,rgbImage,width1,height1);
    readPNG(Dfilename,tempImage,width2,height2);

    if(width1!=width2 || height1!=height2){
      cerr<<"PNG mismatch on "<<RGBfilename<<" and "<<Dfilename<<endl;
    }
    vector<unsigned char> dImage;
    int counter = 1;
    for(vector<unsigned char>::iterator it = tempImage.begin(); it!=tempImage.end();++it){
      if(counter%4==1)
        dImage.push_back(*it);
      counter++;
    }
    if(rgbImage.size()!=width1*height1*4 || dImage.size()!=width2*height2){
      cerr<<"Vector size wrong on "<<RGBfilename<<" and "<<Dfilename<<endl;
    }
    vector<unsigned char> label;
    for(size_t i=1;i<=width2*height2*4; i++){
      if(i%4==0)
        label.push_back((unsigned char)255);
      else
        label.push_back((unsigned char)0);
    }
    labelByDepth(label,dImage,width2,height2);
    string result_filename = RESULT_FILENAME+"_"+to_string(i)+"_label.png";

    savePNG(result_filename, label, width2, height2);
  }
}

void labelByDepth(vector<unsigned char>& label, vector<unsigned char>& dImage, unsigned width, unsigned height){
  vector<size_t> pickList;
  for(size_t idx= 0; idx<dImage.size(); idx++){
    pickList.push_back(idx);
  }
  random_shuffle ( pickList.begin(), pickList.end() );
  unsigned char labelNum = 252;
  while(pickList.size()!=0){
    cout<<"Label:"<<int(labelNum)<<" "<<pickList.size()<<endl;;
    size_t randomSeed = pickList[0];
    unsigned char startD = dImage[randomSeed];
    //pickNum_used.find(randomSeed)->second = true;
    //pickList.pop()//just pop naively
    findNeighbor(label,dImage,pickList,width,height,startD,randomSeed,labelNum);
    labelNum+=3;
  }
}

void findNeighbor(vector<unsigned char>& label, vector<unsigned char>& dImage, vector<size_t>& pickList, unsigned width, unsigned height
                  , unsigned char startD,size_t pickNum ,unsigned char labelNum){
  label[4*pickNum] = labelNum;
  label[4*pickNum+1] = labelNum;
  label[4*pickNum+2] = labelNum;
  for(size_t i=0;i<pickList.size();i++){
    if(pickList[i]==pickNum){
      pickList.erase(pickList.begin()+i);
      break;
    }
  }
  int W = pickNum%width;
  int H = pickNum/width;
  if(W!=0){
    unsigned char leftD = dImage[pickNum-1];
    size_t newNum = pickNum-1;
    if( abs(int(startD)-int(leftD)) <= 1 && checkNeverUsed(newNum,pickList)){
      findNeighbor(label,dImage,pickList,width,height,leftD,newNum,labelNum);
    }
  }
  if(W!=int(width-1)){
    unsigned char rightD = dImage[pickNum+1];
    size_t newNum = pickNum+1;
    if( abs(int(startD)-int(rightD)) <= 1  && checkNeverUsed(newNum,pickList)){
      findNeighbor(label,dImage,pickList,width,height,rightD,newNum,labelNum);
    }
  }
  if(H!=0){
    unsigned char upD = dImage[pickNum-1*width];
    size_t newNum = pickNum-1*width;
    if( abs(int(startD)-int(upD)) <= 1  && checkNeverUsed(newNum,pickList)){
      findNeighbor(label,dImage,pickList,width,height,upD,newNum,labelNum);
    }
  }
  if(H!=int(height-1)){
    unsigned char downD = dImage[pickNum+1*width];
    size_t newNum = pickNum+1*width;
    if( abs(int(startD)-int(downD)) <= 1  && checkNeverUsed(newNum,pickList)){
      findNeighbor(label,dImage,pickList,width,height,downD,newNum,labelNum);
    }
  }
  return;
}

bool checkNeverUsed(size_t newNum, vector<size_t>& pickList){
  bool b = false;
  for(size_t i=0;i<pickList.size();i++){
    if(pickList[i]==newNum){
      b = true;
      break;
    }
  }
  return b;
}

void readPNG(const string filename, vector<unsigned char>& image, unsigned& width, unsigned& height)
{
  unsigned error = lodepng::decode(image, width, height, filename.c_str());
  //if there's an error, display it
  if(error) cout << "decoder error " << error << ": " << lodepng_error_text(error) << endl;
  //the pixels are now in the vector "image", 4 bytes per pixel, ordered RGBARGBA..., use it as texture, draw it, ...
}

void savePNG(const string filename, vector<unsigned char>& image, unsigned width, unsigned height)
{
  //Encode the image
  unsigned error = lodepng::encode(filename.c_str(), image, width, height);

  //if there's an error, display it
  if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
}