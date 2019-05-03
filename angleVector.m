function vector = angleVector(theta,phi)
    vector=[0; 0; 1;];
    rotationMatrixY=[cosd(theta) 0 sind(theta); 0 1 0; -sind(theta) 0 cosd(theta)];
    rotationMatrixX=[1 0 0; 0 cosd(phi) -sind(phi); 0 sind(phi) cosd(phi)];
    vector=rotationMatrixX*vector;
    vector=rotationMatrixY*vector;
end