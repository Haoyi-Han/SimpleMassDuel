%% load
clear all
tracemass = importfile("trace_mass.dat");
pointlist = importfile1("point_list.dat");
%% draw fixed objects
for i=1:size(pointlist)
    viscircles([pointlist.x(i), pointlist.y(i)], pointlist.r(i));
    hold on;
end
%% plot
plot(tracemass.x, tracemass.y);
pbaspect([1, 1, 1]);
hold off;
f = gcf;
exportgraphics(f, "plt.png", "Resolution", 1080);
%% animated video
for i=1:size(pointlist)
    viscircles([pointlist.x(i), pointlist.y(i)], pointlist.r(i));
    hold on;
end
theta = 1;
t=timer('TimerFcn',['theta=theta+1;',...
                    'plot(tracemass.x(1:theta),tracemass.y(1:theta));hold on;',...
                    'axis([-10, 110, -10, 110])'],...
        'ExecutionMode','fixedDelay',...
        'Period',1,...
        'TasksToExecute',Inf);
start(t);