%fitPlane3.m  WCRose 20210908
%Fit a plane through M points by four methods and display results.
%Methods: 
%OLS 1: ordinary least squares with fitlm() from Stats toolbox
%OLS 2: ordinary least squares without Stats toolbox
%TLS 1: total least squares with pca() from Stats toolbox
%TLS 2: total least squares without Stats toolbox
%For Gaetano Pavone.
%Results on console show that planes found by OLS1 and OLS2 are identical,
%and planes found by TLS1 and TLS2 are identical, and the OLS and TLS 
%planes are different.  OLS plane has smaller RMS error in z-direction.
%TLS plane has smaller total (orthogonal) RMS error.
%This version, fitPlane3.m, fits one set of m points.

clear;

%Make the data to fit
nHat=[1/sqrt(3);1/sqrt(3);1/sqrt(3)];           %unit normal, if no noise
zInt=5;                                         %z-axis intercept of the plane, if no noise
%X=[1,-1;1,0;0,1;-1,1;-1,0;0,-1];               %x,y for six points; X~6x2
X=[11,9;11,10;10,11;9,11;9,10;10,9];            %x,y for six points; X~6x2
%X=[1,-1;1,1;-1,1;-1,-1];                       %x,y for four points; X~4x2
m=length(X);                                    %number of points
z=-X*nHat(1:2)/nHat(3)+zInt;                    %z values, before adding noise
sigma=0.1;                                      %noise standard deviation
z=z+sigma*randn(m,1);                           %add noise to the z values

%Method OLS 1: Ordinary least squares, using fitlm() from Statistics Toolbox
mdl=fitlm(X,z);         %fit linear model using Stats toolbox
bMdl=mdl.Coefficients.Estimate; %coefficients from fitlm()
                        %fitlm() return intercept first, then slopes
denom1=sqrt(bMdl(2)^2+bMdl(3)^2+1);
nHatEst1=-[bMdl(2);bMdl(3);-1]/denom1;          %leading "-" so n points up
zInt1=bMdl(1);                                  %z-axis intercept
zFitted1=mdl.Fitted;                            %z-values on plane exactly above or below
rmsez1=sqrt(sum((zFitted1-z)'*(zFitted1-z))/m); %RMS error in z-direction
d=([X,z]-ones(m,1)*mean([X,z]))*nHatEst1;       %vector of distances from points to plane
rmset1=sqrt(d'*d/m);                            %total (orthogonal) RMS error    

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
rmset2=sqrt(d'*d/m);                            %total (orthogonal) RMS error

%Method TLS 1: Total least squares by principal component analysis, uses Stats Toolbox
coeff=pca([X,z]);                               %prinicpal components
nHatEst3=coeff(:,3);                            %nHat=3rd p.c.
zInt3=[Xm,zm]*nHatEst3/nHatEst3(3);             %z-axis intercept
zFitted3=zInt3-X*nHatEst3(1:2)/nHatEst3(3);     %z-values on plane exactly above or below
rmsez3=sqrt(sum((zFitted3-z)'*(zFitted3-z))/m); %RMS error in z-direction
d=([X,z]-ones(m,1)*mean([X,z]))*nHatEst3;       %vector of distances from points to plane
rmset3=sqrt(d'*d/m);                            %total (orthogonal) RMS error

%Method TLS 2: Total least squares _without_ using Stats Toolbox
[V,D]=eig(cov([X z]));                          %V=eigenvectors, D=eigenvalues, unsorted
[~,jmin]=min(diag(D));                          %index of column with smallest eigenvalue
nHatEst4=V(:,jmin)*sign(V(3,jmin));             %nHat=eigenvector with smallest eigenvalue
zInt4=[Xm,zm]*nHatEst4/nHatEst4(3);             %z-axis intercept
zFitted4=zInt4-X*nHatEst4(1:2)/nHatEst4(3);     %z-values on plane exactly above or below
rmsez4=sqrt(sum((zFitted4-z)'*(zFitted4-z))/m); %RMS error in z-direction
d=([X,z]-ones(m,1)*mean([X,z]))*nHatEst4;       %vector of distances from points to plane
rmset4=sqrt(d'*d/m);                            %total (orthogonal) RMS error

%Display results
fprintf('Points to fit, each point is a column.\n');
disp([X';z']);
fprintf('Method          nHat            zIntercept  RMSEz   RMSEtot\n');
fprintf('OLS 1: [%.4f, %.4f, %.4f] %8.5f  %8.5f %8.5f\n',nHatEst1,zInt1,rmsez1,rmset1);
fprintf('OLS 2: [%.4f, %.4f, %.4f] %8.5f  %8.5f %8.5f\n',nHatEst2,zInt2,rmsez2,rmset2);
fprintf('TLS 1: [%.4f, %.4f, %.4f] %8.5f  %8.5f %8.5f\n',nHatEst3,zInt3,rmsez3,rmset3);
fprintf('TLS 2: [%.4f, %.4f, %.4f] %8.5f  %8.5f %8.5f\n',nHatEst4,zInt4,rmsez4,rmset4);
