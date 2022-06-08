% https://www.mathworks.com/matlabcentral/answers/1449234-z-angle-of-a-fit-plane-having-4-points
function results = plane_fit_tls(points)

results = struct;
X = points(:,1:2);
z = points(:,3);

m=length(X);                                    %number of points

%Method TLS 2: Total least squares _without_ using Stats Toolbox
Xm=mean(X); 
zm=mean(z);
Xc=X-Xm;                    %centered X=[x,y] data
zc=z-zm;                    %centered z data
[V,D]=eig(cov([X z]));                          %V=eigenvectors, D=eigenvalues, unsorted
[~,jmin]=min(diag(D));                          %index of column with smallest eigenvalue
nHatEst4=V(:,jmin)*sign(V(3,jmin));             %nHat=eigenvector with smallest eigenvalue
zInt4=[Xm,zm]*nHatEst4/nHatEst4(3);             %z-axis intercept
zFitted4=zInt4-X*nHatEst4(1:2)/nHatEst4(3);     %z-values on plane exactly above or below
rmsez4=sqrt(sum((zFitted4-z)'*(zFitted4-z))/m); %RMS error in z-direction
d=([X,z]-ones(m,1)*mean([X,z]))*nHatEst4;       %vector of distances from points to plane
rmset4=sqrt(d'*d/m);                            %total (orthogonal) RMS error


results.normal = nHatEst4;
results.rmse_z = rmsez4;
results.rmse_tot = rmset4;

end