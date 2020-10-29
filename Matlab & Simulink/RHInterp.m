clear
clc

%% user defined variables
rh_rim = [31 61 60 25]; %humidity readings @ 60 165 270 345 degree
rh_center=44; %RH @ the center
degree=[60 165 270 345];

%% interpolate around the rim
r=[rh_rim rh_rim rh_rim];
d=[degree degree+360 degree+720];
L=length(r);
m=(r(2:L)-r(1:L-1))./(d(2:L)-d(1:L-1)); %slopes between each point
gap=d(2:L)-d(1:L-1);

aa=rh_rim(1)*ones(1,degree(1));
bb=rh_rim(2)*ones(1,degree(2)-degree(1));
cc=rh_rim(3)*ones(1,degree(3)-degree(2));
dd=rh_rim(4)*ones(1,degree(4)-degree(3));
ee=rh_rim(1)*ones(1,360-degree(4));
RH=[aa bb cc dd ee];
RH=[RH RH RH];
DEGREE=1:360*3;

for i=2:L
    ff=d(i-1)+1:1:d(i);
    ff=ff-d(i);
    ff=ff*m(i-1);
    RH(d(i-1)+1:1:d(i))=RH(d(i-1)+1:1:d(i))+ff;
end

RH=RH(361:720);
DEGREE=DEGREE(361:720)-360;

figure
plot(DEGREE,RH); hold on
plot(d,r,'d')
xlim([0 360])
xlabel('position') 
ylabel('RH') 

%% define domain
def=180;
R=150; %mm
X=R*cos(DEGREE*pi/180);
Y=R*sin(DEGREE*pi/180);
X=[X 0];
Y=[Y 0];
RH=[RH rh_center];

%% setup
x = X;
y = Y;
z = RH;
[xq, yq] = meshgrid(-def:1:def,-def:1:def); %create points

%% linear interpolation
rhcg = griddata(x,y,RH,xq,yq);

%% cubic interpolation
figure
s = surf(xq,yq,rhcg);
length = 100;

%% five colors
color1 = [239 61 37]/255;
color2 = [252 176 23]/255;
color3 = [198 218 76]/255;
color4 = [90 170 220]/255;
color5 = [0 0 255]/255;
allcolor = [color1;color2;color3;color4;color5];
colors_a=[];
colors_p=ones(3,length*4);
for i=1:3
    for j=1:4
column=linspace(allcolor(j,i),allcolor(j+1,i),length);
colors_a=[colors_a column];
    end
    colors_p(i,:)=colors_a;
    colors_a=[];
end
colors_p=colors_p';

%% plot
colormap(colors_p)
colorbar
caxis([20 65])
view(0,90)
s.EdgeColor = 'none';
s.FaceAlpha = 1;
