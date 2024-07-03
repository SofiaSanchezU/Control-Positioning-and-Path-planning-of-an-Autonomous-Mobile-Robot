function [points_x, points_y, time_traj] = visible_graph(start,goal)
tic
%% Defining environment variables
% % start = [40, 10];     % start position
% % goal = [85, 85];   % goal position
% % n = 4;             % no. of obstacles

%% Defining the enviroment
figure(2)
title('Obstacle Managment')
rectangle('Position',[20 20 15 10],'Curvature',1 , 'FaceColor',[0.3010 0.7450 0.9330])
axis([0 100 0 100])
axis square
rectangle('Position',[60 10 35 35], 'Curvature',[1 1] , 'FaceColor',[0.3010 0.7450 0.9330])
P = [15 50; 25 60; 30 85; 25 95; 15 95];
pgon = polyshape(P);
P1 = [45 65; 55 65; 55 60; 65 60; 65 75; 45 75];
pgon1 = polyshape(P1);
hold on
plot(pgon,'FaceColor',[0.2010 0.5450 0.9]) 
plot(pgon1,'FaceColor',[0.2010 0.5450 0.9])
% Plotting start position
circles(start(1), start(2),2, 'facecolor','green')
% Plotting goal position
circles(goal(1), goal(2),2, 'facecolor','yellow')
hold off
%%%%%%%%%%%
%% Give the same shape to the obstacles
pause(1)
figure(2)
title('Obstacle Managment: Outer Approximation')
rectangle('Position',[20 20 15 10], 'FaceColor',[0 1 1])
rectangle('Position',[20 20 15 10],'Curvature',1 , 'FaceColor',[0.3010 0.7450 0.9330])
rectangle('Position',[60 10 35 35], 'FaceColor',[0 1 1])
rectangle('Position',[60 10 35 35], 'Curvature',[1 1] , 'FaceColor',[0.3010 0.7450 0.9330])
% rectangle('Position',[45 65 10 10], 'FaceColor',[0.3010 0.7450 0.9330],'EdgeColor',[0.3010 0.7450 0.9330])
% rectangle('Position',[55 60 10 15], 'FaceColor',[0.3010 0.7450 0.9330],'EdgeColor',[0.3010 0.7450 0.9330])
rectangle('Position',[45 60 20 15], 'FaceColor',[0 1 1])
rectangle('Position',[15 50 15 45], 'FaceColor',[0 1 1])
P = [15 50; 25 60; 30 85; 25 95; 15 95];
pgon = polyshape(P);
P1 = [45 65; 55 65; 55 60; 65 60; 65 75; 45 75];
pgon1 = polyshape(P1);
hold on
plot(pgon,'FaceColor',[0.2010 0.5450 0.9]) 
plot(pgon1,'FaceColor',[0.2010 0.5450 0.9])
% Plotting start position
circles(start(1), start(2),2, 'facecolor','green')
% Plotting goal position
circles(goal(1), goal(2),2, 'facecolor','yellow')
hold off

%%%%%%%%%%%
%% Oversize the obstacles
pause(1)
figure(2)
title('Obstacle Management: Enlarge Size')
rectangle('Position',[15 15 25 20], 'FaceColor',[0 0.4470 0.7410])
axis([0 100 0 100])
axis square
rectangle('Position',[55 5 45 45], 'FaceColor',[0 0.4470 0.7410])
rectangle('Position',[40 55 30 25], 'FaceColor',[0 0.4470 0.7410])
rectangle('Position',[10 45 25 55], 'FaceColor',[0 0.4470 0.7410])
rectangle('Position',[20 20 15 10], 'FaceColor',[0 1 1])
rectangle('Position',[20 20 15 10],'Curvature',1 , 'FaceColor',[0.3010 0.7450 0.9330])
rectangle('Position',[60 10 35 35], 'FaceColor',[0 1 1])
rectangle('Position',[60 10 35 35], 'Curvature',[1 1] , 'FaceColor',[0.3010 0.7450 0.9330])
% rectangle('Position',[45 65 10 10], 'FaceColor',[0.3010 0.7450 0.9330],'EdgeColor',[0.3010 0.7450 0.9330])
% rectangle('Position',[55 60 10 15], 'FaceColor',[0.3010 0.7450 0.9330],'EdgeColor',[0.3010 0.7450 0.9330])
rectangle('Position',[45 60 20 15], 'FaceColor',[0 1 1])
rectangle('Position',[15 50 15 45], 'FaceColor',[0 1 1])
P = [15 50; 25 60; 30 85; 25 95; 15 95];
pgon = polyshape(P);
P1 = [45 65; 55 65; 55 60; 65 60; 65 75; 45 75];
pgon1 = polyshape(P1);
hold on
plot(pgon,'FaceColor',[0.2010 0.5450 0.9]) 
plot(pgon1,'FaceColor',[0.2010 0.5450 0.9])
% Plotting start position
circles(start(1), start(2),2, 'facecolor','green')
% Plotting goal position
circles(goal(1), goal(2),2, 'facecolor','yellow')
hold off


%% initialising the hash map

keys = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r'};
values = {start, [15,15], [40,15], [40,35], [15,35], [55,5], [100,5], [100,50], [55,50], [40,55], [70,55], [70,80], [40,80], [10,45], [35,45], [35,100], [10,100], goal};

Map = containers.Map(keys, values);
Map('j');


%% Building all the obstacle edges
length1= size(keys);    % this contains the number of nodes

edges = [];

for i = 2:(length1(2)-2)   
        temp = [values{1,i}(1), values{1,i+1}(1), values{1,i}(2), values{1,i+1}(2)];
        edges = vertcat(edges, temp);                
end


% Removing edges which are not obstacle edges
sizeEdges = size(edges);
i = 4;
while i < (sizeEdges(1)-1)
    [edges,ps] = removerows(edges,'ind',i);
    sizeEdges = size(edges);
    i = i + 3;
end

% Adding 1 edge pair in each obstacle which were not added in the earlier
% for loop
i = 2;
while i < length1(2)-2    
    temp = [values{1,i}(1), values{1,i+3}(1), values{1,i}(2), values{1,i+3}(2)];
    edges = vertcat(edges, temp);
    i = i + 4;    
end


%% Calculating the valid edges and adding them to the graph

ledgeSize = size(edges);
noEdges = ledgeSize(1);

G = graph();

for i = 1:length1(2)    
    for j = (i + 1):length1(2)        
        % find equation of the edge to be checked
        p1 = values{1,i};
        q1 = values{1,j};
        m1 = (q1(2)-p1(2))/(q1(1)-p1(1));
        c1 = p1(2) - m1*(p1(1));
        %%%%%%%%%%%%
    
        flag = 1;    % flag to check if the edge has any intersection with any other edge;
                     % '1' means no intersection
        % need to compare with the edges   
        for k = 1:noEdges
            
            ed = edges(k,:);            
            m2 = (ed(4) - ed(3))/(ed(2) - ed(1));
            if(ed(2)==ed(1))
                m2 = 1e+10;
            end
            c2 = ed(3) - m2*ed(1);  
            
            if m1==m2 %% ignoring 
                t = 1;
            else
                
                %%%%%%
                temp1 = ed(3) - m1*ed(1) - c1;
                temp2 = ed(4) - m1*ed(2) - c1;
                
                temp3 = p1(2) - m2*p1(1) - c2;
                temp4 = q1(2) - m2*q1(1) - c2;
                
                if (sign(temp1) ~= sign(temp2)) &&  sign(temp1)~=0 && sign(temp2)~=0 && (sign(temp3) ~= sign(temp4)) &&  sign(temp3)~=0 && sign(temp4)~=0
                    flag = 0;
                    break
                end
                %%%%%%
            end

        end
        if flag==1
            G = addedge(G,keys{i}, keys{j});
        end
                
    end    
end

%% Removing the diagonals of the obstacle from the visible edges
length1 = size(keys);    % this contains the number of nodes
i = 2;

while i < (length1(2)-2)
    
    G = rmedge(G,keys{i}, keys{i+2});
    G = rmedge(G,keys{i+1}, keys{i+3});
    i = i + 4;
end

% G = rmedge(G,keys{15}, keys{32});
% G = rmedge(G,keys{14}, keys{33});
% G = rmedge(G,keys{15}, keys{4});
% G = rmedge(G,keys{14}, keys{5});
% G = rmedge(G,keys{10}, keys{25});
% G = rmedge(G,keys{9}, keys{12});

%% Plotting all the visible edges

visEd = G.Edges;  % this has the visible edges
sizeEd = size(G.Edges);
pause(3)
figure(2)
title('Visibility Graph')
rectangle('Position',[15 15 25 20], 'FaceColor',[0 0.4470 0.7410])
axis([0 100 0 100])
axis square
rectangle('Position',[55 5 45 45], 'FaceColor',[0 0.4470 0.7410])
rectangle('Position',[40 55 30 25], 'FaceColor',[0 0.4470 0.7410])
rectangle('Position',[10 45 25 55], 'FaceColor',[0 0.4470 0.7410])
rectangle('Position',[20 20 15 10], 'FaceColor',[0 1 1])
rectangle('Position',[20 20 15 10],'Curvature',1 , 'FaceColor',[0.3010 0.7450 0.9330])
rectangle('Position',[60 10 35 35], 'FaceColor',[0 1 1])
rectangle('Position',[60 10 35 35], 'Curvature',[1 1] , 'FaceColor',[0.3010 0.7450 0.9330])
% rectangle('Position',[45 65 10 10], 'FaceColor',[0.3010 0.7450 0.9330],'EdgeColor',[0.3010 0.7450 0.9330])
% rectangle('Position',[55 60 10 15], 'FaceColor',[0.3010 0.7450 0.9330],'EdgeColor',[0.3010 0.7450 0.9330])
rectangle('Position',[45 60 20 15], 'FaceColor',[0 1 1])
rectangle('Position',[15 50 15 45], 'FaceColor',[0 1 1])
P = [15 50; 25 60; 30 85; 25 95; 15 95];
pgon = polyshape(P);
P1 = [45 65; 55 65; 55 60; 65 60; 65 75; 45 75];
pgon1 = polyshape(P1);
hold on
plot(pgon,'FaceColor',[0.2010 0.5450 0.9]) 
plot(pgon1,'FaceColor',[0.2010 0.5450 0.9])
% Plotting start position
circles(start(1), start(2),2, 'facecolor','green')
% Plotting goal position
circles(goal(1), goal(2),2, 'facecolor','yellow')



for i=1:sizeEd(1)
   x = visEd(i,1);
   xx = x{1,1};
   p1 = Map(xx{1,1});
   p2 = Map(xx{1,2});
   xpoints = [p1(1,1), p2(1,1)];
   ypoints = [p1(1,2), p2(1,2)];
   hold on
   plot(xpoints, ypoints, 'b')
   
end
hold off
%% finding the shortest path in the graph and printing it 

path = shortestpath(G, keys{1}, keys{length1(2)});
pathSize = size(path);

totalDis = 0;    % this will contain the total distance in units for the final selected path

for i=1:pathSize(2)-1
    p1 = Map(path{i});
    p2 = Map(path{i+1});
    
    totalDis = totalDis + EuclDist(p1,p2);
    
    xpoints = [p1(1,1), p2(1,1)];
    ypoints = [p1(1,2), p2(1,2)];
    hold on
    plot(xpoints, ypoints, 'k', 'LineWidth', 3)
    title('Visibility Graph')
end
points=[];
t_start=0;
time_traj=[t_start];
t_p_p=0;
for l=1:length(path);
for m=1:length(keys);
   if strcmp(path(l),keys(m))
    points1=[values(m)];
   else 
    points1=[]; 
   end
    points=[points points1];
end
end
for e=1:length(points)-1;
distance_p_p=sqrt((points{e+1}(1)-points{e}(1))^2+(points{e+1}(2)-points{e}(2))^2);
t_p_p=t_p_p+distance_p_p/10;
time_traj=[time_traj t_p_p];
end
points = cell2mat(points);
points_x=[];
points_y=[];
for o=1:2:length(points)-1;
p_x=points(o);
points_x=[points_x p_x];
p_y=points(o+1);
points_y=[points_y p_y];
end
end
%%%%%%%%%%%%%% end %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

