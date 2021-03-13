%% deal with angle wrap-around
function angle_new = wrappedAngle(angle)

if angle < 0 
    angle_new = angle + 2*pi;
else
    angle_new = angle;
end