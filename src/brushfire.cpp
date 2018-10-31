#include"brushfire.h"



brushfire::brushfire()
{

}


void brushfire::makebrushfire()
{
   image2=getimage();
   int height = image2.cols;
   int width = image2.rows;
   int WorldArray[width][height];

   Vec3b BGR_BLACK ={0,0,0};
   for(int row =0; row< width; row++)
   {
       for(int col=0; col<height;col++)
       {
           Vec3b BGR = image2.at<Vec3b>(row, col);
           if(BGR == BGR_BLACK)
           {
               WorldArray[row][col]=1;
           }
           else{
               WorldArray[row][col]=0;
           }
       }
   }

   cout << "Worl eviroment" <<endl;

   for(int row =0; row < width;row++)
   {
       for(int col=0; col < height;col++)
       {
           cout<< WorldArray[row][col]<< " ";
       }
       cout <<endl;
   }
//BRUSHFIRE ALGORITHM
  int elementSearchValue=1;
  bool zeroElPres = true; //zero element present
   cout<<"Brushfire intialised"<<endl;

   while (zeroElPres)
   {

       bool zeroELDec =false; //zero elemten detected
       for(int row=0; row < width; row++)
       {
           for(int col=0; col < height; col++)
           {

               if(WorldArray[row][col] == 0)
               {
                   //Decteced zero element in the array
                   zeroELDec = true;
                   zeroElPres = true;
                   //Initialise nabo values
                   int naboarray[8] = {0};

                  if(row-1 >=0)
                  {
                      naboarray[0] = WorldArray[row-1][col];
                  }
                  if(row+1 >=0)
                  {
                      naboarray[1] = WorldArray[row+1][col];
                  }
                  if(col-1>=0)
                  {
                      naboarray[2] = WorldArray[row][col-1];
                     if(row-1 >=0)
                      {
                        naboarray[3] = WorldArray[row-1][col-1];
                      }
                     if(row+1>=0)
                     {
                       naboarray[4] = WorldArray[row+1][col-1];
                     }
                  }
                  if(col+1>=0)
                  {
                      naboarray[5] = WorldArray[row][col+1];
                      if(row-1>=0)
                      {
                          naboarray[6] = WorldArray[row-1][col+1];
                      }
                      if(row+1>=0)
                      {
                          naboarray[7] = WorldArray[row+1][col+1];
                      }
                  }
                  bool shValFo = false;
                  for(int nabo =0; !shValFo && nabo<8; nabo++)
                  {
                      if(naboarray[nabo] == elementSearchValue)
                      {
                          WorldArray[row][col] = elementSearchValue+1;
                          shValFo = true;
                      }
                  }

               }
                if(!zeroELDec)
                   {
                       zeroElPres = false;

                   }

           }
           if(!zeroELDec)
               zeroElPres= false;
       }

       elementSearchValue++;

   }
    cout<<"burshfire done"<<endl;
   cout<< '\n';
   cout<<"World enviroment with brushfire:" <<endl;

   for(int row=0; row < width; row++)
   {
       for(int col =0; col <height; col++)
       {
           cout<<WorldArray[row][col]<<" ";
       }
       cout<<endl;
   }
namedWindow("Display world", WINDOW_KEEPRATIO);
imshow("Display world",image2);
waitKey(0);



}

Mat brushfire::getimage()
{
    image = imread("/home/thor/MarbleFinder/world/models/bigworld/meshes/floor_plan.png",IMREAD_COLOR);
    //image = imread("/home/thor/MarbleFinder/world/models/smallworld/meshes/floor_plan.png", IMREAD_COLOR);

    return image;

}

void brushfire::showbrushfire()
{

}



brushfire::~brushfire()
{

}
