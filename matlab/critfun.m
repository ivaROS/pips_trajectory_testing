function [] = critfun(XTRAIN,ytrain,XTEST,ytest)



critfun = @(X,Y)...
      (sum(~strcmp(yt,classify(Xt,XT,yT,'quadratic'))));
    
end