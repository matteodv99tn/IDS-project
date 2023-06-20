clc; clear all; close all;
rng default;

X = [-1.5 3.2; 1.8 3.3; -3.7 1.5; -1.5 1.3; 0.8 1.2; ...
      3.3 1.5; -4.0 -1.0; -2.3 -0.7; 0 -0.5; 2.0 -1.5; ...
      3.7 -0.8; -3.5 -2.9; -0.9 -3.9; 2.0 -3.5; 3.5 -2.25];
[VX,VY] = voronoi(X(:,1),X(:,2));
h = plot(VX,VY,'-b',X(:,1),X(:,2),'.r');
xlim([-4,4])
ylim([-4,4])
dt =  delaunayTriangulation(X);
[V,R] = voronoiDiagram(dt);
axis equal

w = warning('off','all');

tic
[V,R,XY] = VoronoiLimit(X(:,1), X(:,2),'figure','off');
toc

P = [0 0];
hold on;
plot(P(1),P(2),'sk');
for i=1:numel(R)
Poly = V(R{i},:);
if inpolygon(P(1),P(2),Poly(:,1),Poly(:,2))
    fprintf("%d - %.2f %.2f inside\n", i, XY(i,1), XY(i,2));
else
    fprintf("%d - %.2f %.2f outside\n", i, XY(i,1), XY(i,2));
end
end
warning(w);


