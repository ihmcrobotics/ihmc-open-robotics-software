function [cloud color]= getACloud(sub)

    sub.setOnNewMessageListeners({@(x) assignin('base','cloud_',x)});
    
    %wait for cloud
    while 1
        try
            cloud_=evalin('base','cloud_;');
            break;
        catch
            fprintf(1,'.');
            pause(0.1)
        end
    end

    numPoints = cloud_.getWidth()*cloud_.getHeight();
    pointStep = cloud_.getPointStep;
    offset = cloud_.getData().arrayOffset();
    byteBuffer=typecast(cloud_.getData().array,'uint8');
    byteBufferCropped=byteBuffer(offset+1:(offset+numPoints*pointStep));
    data=reshape(typecast(byteBufferCropped,'single'), 4,[]);

    %XYZ
    cloud = double(data(1:3,:));

    %color
    colorType=cloud_.getFields.get(3).getName();    
    if  strcmp(colorType, 'luminance')
        color=data(4,:);
    elseif strcmp(colorType, 'rgb')
        colorBuffer=reshape(typecast(data(4,:),'uint8'),4,[]);
        color = double(colorBuffer([3 2 1],:))/255.0;
    else
        error ('no such color type');
    end
        

    sub.setOnNewMessageListeners({});