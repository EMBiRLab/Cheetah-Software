% https://www.mathworks.com/matlabcentral/answers/1449234-z-angle-of-a-fit-plane-having-4-points
function results = plane_fit_ols(points)

results = struct;
X = points(:,1:2);
z = points(:,3);

m=length(X);                                    %number of points

%Method OLS 2: Ordinary least squares _without_ using Statistics Toolbox
Xm=mean(X);
zm=mean(z);
Xc=X-Xm;                    %centered X=[x,y] data
zc=z-zm;                    %centered z data
b=(Xc'*Xc)\(Xc'*zc);        %estimate regression coefficients [b1;b2]
denom2=sqrt(b(1)^2+b(2)^2+1);
nHatEst2=-[b(1);b(2);-1]/denom2;                %leading "-" so n points up
zInt2=zm-Xm*b;                                  %z-axis intercept
zFitted2=Xc*b+zm;                               %z-values on plane exactly above or below
rmsez2=sqrt(sum((zFitted2-z)'*(zFitted2-z))/m); %RMS error in z-direction
d=([X,z]-ones(m,1)*mean([X,z]))*nHatEst2;       %vector of distances from points to plane
rmset2=sqrt(d'*d/m);  

results.normal = nHatEst2;
results.rmse_z = rmsez2;
results.rmse_tot = rmset2;

end