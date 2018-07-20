clear all
%% Create the working space.
bellyA=[-170 -360];
bellyB=[170 -360];
bellyC=[170 -610];
bellyD=[-160 -610];
bellyE=[-85 -515];
 

global X_MIN X_MAX Y_MIN Y_MAX;
X_MIN=-185;
X_MAX=170;
Y_MIN=-615;
Y_MAX=-350;

global whiteImage
whiteImage = 255 * ones(265, 350, 'uint8');
figure(1)
% imshow(whiteImage);


% axis([X_MIN X_MAX Y_MIN Y_MAX]);
global bellyF bellyF_radius
bellyF=[170 -485]; % origin of circle 
bellyF_radius= 125;
eq_pos_circle=[bellyF(1)-bellyF_radius bellyF(2)-bellyF_radius 2*bellyF_radius 2*bellyF_radius];
% plot the result 
figure(1)
plot([bellyA(1) bellyB(1)],[bellyA(2) bellyB(2)],'-bs', 'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5]);
hold on
plot([bellyA(1) bellyE(1)],[bellyA(2) bellyE(2)],'-bs', 'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5]);
hold on
plot([bellyD(1) bellyE(1)],[bellyD(2) bellyE(2)],'-bs', 'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5]);
hold on
plot([bellyC(1) bellyD(1)],[bellyC(2) bellyD(2)],'-bs', 'MarkerSize',10,'MarkerEdgeColor','b','MarkerFaceColor',[0.5,0.5,0.5]);
hold on
rectangle('Position',eq_pos_circle,'Curvature',[1 1],'EdgeColor','b');
axis([X_MIN X_MAX Y_MIN Y_MAX]);

axis equal

global eqA_E eqD_E eqA_B eqC_D
eqA_E=polyfit([bellyA(1) bellyE(1)],[bellyA(2) bellyE(2)],1); 
eqD_E=polyfit([bellyD(1) bellyE(1)],[bellyD(2) bellyE(2)],1); 
eqA_B=polyfit([bellyA(1) bellyB(1)],[bellyA(2) bellyB(2)],1); 
eqC_D=polyfit([bellyC(1) bellyD(1)],[bellyC(2) bellyD(2)],1); 


%% fill the region and mark the boundary 
% free space = 1
% boundary = 2
% obstacle = 0

% fill the region
for i=X_MIN:X_MAX
   for j= Y_MIN:Y_MAX
       if(ToCheckIfInObstacleSpace(i,j))
           % is a free space, mark as 1
           whiteImage(j+616,i+186)=0;
       else
           whiteImage(j+616,i+186)=125;
       end
   end
end


for i=X_MIN+1:X_MAX
   for j= Y_MIN+1:Y_MAX
       if((whiteImage(j+616,i+186)==0 && whiteImage(j+616-1,i+186)==125) ...
          || (whiteImage(j+616,i+186)==125 && whiteImage(j+616-1,i+186)==0)...
          || (whiteImage(j+616,i+186)==0 && whiteImage(j+616,i+186-1)==125)...
          || (whiteImage(j+616,i+186)==125 && whiteImage(j+616,i+186-1)==0))
           % is a boundary, mark as 255
           whiteImage(j+616,i+186)=255;
       end
   end
end




%% morse decomposition 
% using h(x)=x from left to right 

% 1. find critical point 
criticalPoint=0;
connectivity=0;

for i=X_MIN+1:X_MAX
   for j= Y_MIN+1:Y_MAX
       if(whiteImage(j+616,i+186)==255 && whiteImage(j+616+1,i+186)~=255)
            if(CheckIfCritical(i,j))
              whiteImage(j+616,i+186)=50;               
            end
       end
   end
end

% filter some critical point which is too close to each other
CriticalPointIndex=1;
for i=X_MIN+1:X_MAX
   for j= Y_MIN+1:Y_MAX
       if(whiteImage(j+616,i+186)==50)
          if(ToFilteredNoiseCriticalPoint(i,j))
%            CriticalPoint(CriticalPointIndex,:)=[i,j];
           CriticalPoint(CriticalPointIndex,:)=[i+186, j+616];
           CriticalPointIndex=CriticalPointIndex+1;
          end
       end
   end
end



figure(2)
imshow(whiteImage);







%% Zig-zag

% need to have image coordinate
function [OutoFImageRange]=CheckOutoFImageRange(ImageCurrent_X,ImageCurrent_Y)
global whiteImage
OutoFImageRange=false;
[m,n,~]=size(whiteImage);
if(ImageCurrent_X > n || ImageCurrent_X < 1 || ... 
        ImageCurrent_Y < 1 || ImageCurrent_Y > m)
    OutoFImageRange=true;
end

end


function [this_is_true_critical]= ToFilteredNoiseCriticalPoint(Current_X, Current_Y)
global whiteImage
white_count=0; % at least two white point 
cp_point=0;

for i=Current_X-1:Current_X+1
    for j=Current_Y-1:Current_Y+1
        imageY=j+616
        imageX=i+186
      if(~CheckOutoFImageRange(i+186, j+616))
        if(whiteImage(j+616,i+186)==255)
            white_count=white_count+1;
        end
        if(whiteImage(j+616,i+186)==50)
            cp_point=cp_point+1;    
        end
      end
    end
end


if (cp_point<3 && white_count>=2)
    this_is_true_critical=true;
else
    this_is_true_critical=false;
end

end




function [InFreeSpace]=ToCheckIfInObstacleSpace(Current_X, Current_Y)
global bellyF bellyF_radius
global eqA_E eqD_E eqA_B eqC_D

poly_half_A_E=eqA_E(1)*Current_X+eqA_E(2)-Current_Y;
poly_half_D_E=eqD_E(1)*Current_X+eqD_E(2)-Current_Y;

% check for the square obstacle
if(Current_Y <= eqA_B(2) && Current_Y >= eqC_D(2) && ...
      ((Current_X-bellyF(1))^2+(Current_Y-bellyF(2))^2-bellyF_radius^2)>=0  && ... 
       (poly_half_A_E <0 || poly_half_D_E >0) )
    InFreeSpace=true;
else
    InFreeSpace=false;
end

end

function [IsCriticalPoint]=CheckIfCritical(Current_X, Current_Y)
global whiteImage
global  Y_MIN Y_MAX;
up_free=false;
up_obstacle=false;
down_free=false;
down_obstacle=false;

% search up
for j=Current_Y:Y_MAX 
       if(whiteImage(j+616,Current_X+186)==0)
           up_free=true;
           break
       else if (whiteImage(j+616,Current_X+186)==125)
           up_obstacle=true;
           break
           end      
       end
end
% search down
for j=Current_Y:-1:Y_MIN
           if(whiteImage(j+616,Current_X+186)==0)
           down_free=true;
           break
       else if (whiteImage(j+616,Current_X+186)==125)
           down_obstacle=true;
           break
           end      
        end
end
% return whether is a critical point 

if((down_free && up_free) || (up_obstacle && down_obstacle))
    IsCriticalPoint=true;
else
    IsCriticalPoint=false;
end

end

