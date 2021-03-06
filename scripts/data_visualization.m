clc,close all, clear all
data=load("data1.mat");
x=zeros(1,2);
y=zeros(1,2);
z=zeros(1,2);

% renovation_waypaths_onecell=data.renovation_waypaths_onecell;
% figure;
% for i=1:1:size(renovation_waypaths_onecell,1)
%     x(1)=renovation_waypaths_onecell(i,1);
%     x(2)=renovation_waypaths_onecell(i,4);
%     y(1)=renovation_waypaths_onecell(i,2);
%     y(2)=renovation_waypaths_onecell(i,5);
%     z(1)=renovation_waypaths_onecell(i,3);
%     z(2)=renovation_waypaths_onecell(i,6);
%     plot3(x,y,z,'r');
%     hold on;
% end
% hold on;
% axis equal;

selected_waypoints_list=data.selected_waypoints_list;
figure;
for i=1:1:size(selected_waypoints_list,1)-1
    x(1)=selected_waypoints_list(i,1);
    x(2)=selected_waypoints_list(i+1,1);
    y(1)=selected_waypoints_list(i,2);
    y(2)=selected_waypoints_list(i+1,2);
    z(1)=selected_waypoints_list(i,3);
    z(2)=selected_waypoints_list(i+1,3);
    plot3(x,y,z,'k');
    hold on;
end
axis equal;