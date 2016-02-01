clear
clf
% vertex 1 2 3, area
tri = [1 2 3 1]
maxArea=tri(4);
vertex = eye(3);

triToSplit=1;
while maxArea>1e-2
    triVertex = tri(triToSplit,1:3);
    triArea = tri(triToSplit,4);
    
    %split into 4 sub-triangles
    % make new vertexes
    newVertex = [mean(vertex(triVertex([1 2]),:));
                 mean(vertex(triVertex([2 3]),:));
                 mean(vertex(triVertex([1 3]),:))];
    for i=1:3        
        newVertex(i,:) = newVertex(i,:)/norm(newVertex(i,:));
    end
    newVertexId = [1 2 3]'+size(vertex,1);   
    vertex = [vertex;newVertex];
    
    %make new triangles
    newTri = [
        triVertex(1) newVertexId(1) newVertexId(3) triArea/4;
        triVertex(2) newVertexId(1) newVertexId(2) triArea/4;
        triVertex(3) newVertexId(2) newVertexId(3) triArea/4;
        newVertexId(1) newVertexId(2) newVertexId(3) triArea/4;
        ];
    tri(triToSplit,4)=-1;
    tri = [tri ; newTri];
    
    %find next one to split
    [maxArea, triToSplit] = max(tri(:,4));
    maxArea
    scatter3(vertex(:,1), vertex(:,2),vertex(:,3),10,'o');
    
end