# http://faculty.washington.edu/jonno/book/nonparametric2.R
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# BPD example in Section 11.5
#
library(locfit)
myexpit <- function(x,b0,b1){expit <- exp(b0+b1*x)/( 1+exp(b0+b1*x) )}
#bw <- read.table("birthweight.txt",header=T)
bw <- read.table("results.csv",header=T)
bw <- bw[seq(1,100),]
birthweight <- bw$birthweight <- bw$chanceConstraint
#BPD <- bw$BPD <- (1-bw$kickFailure) * (1-bw$collisionFailure) * (1-bw$kickSuccess)
#BPD <- bw$BPD <- (1-kickFailure) * (1-collisionFailure) * (1-kickSuccess)
#BPD <- bw$BPD <- bw$kickFailure
BPD <- bw$BPD <- bw$kickSuccess
#BPD <- bw$BPD <- bw$kickSuccess
#attach(bw)
lrmod1 <- glm(BPD~birthweight,family=binomial)
x <- seq(min(birthweight),max(birthweight))
y <- bw$BPD
meanbw <- mean(bw$birthweight)
sdbw <- sd(bw$birthweight)
x <- (bw$birthweight-meanbw)/sdbw
#
# My own AIC function
#
aicbinfun <- function(y,n,p,df){
  ndat <- length(y)
  dfterm <- 2*df
  loglik <- 0
  for (i in 1:ndat){
    loglik <- loglik
            + lchoose(n[i],y[i])
            + y[i]*log(p[i])
            + (n[i]-y[i])*log(1-p[i])
  }
  aicbin <- (-2*loglik  + dfterm)
  list(aicbin=aicbin,dfterm=dfterm,loglik=loglik)
}
n <- rep(1,length(y))
hval <- seq(0.1,10,.1) # seq(1,5,.1)
aicval <- myaicval <- df <- loglik <- NULL
for (i in 1:length(hval)){
  locfitbw <- locfit(y~lp(x,deg=1,nn=0,h=hval[i]),data=bw,family="binomial")
  preds <- predict(locfitbw,newdata=x)
  df[i] <- locfitbw$dp['df1'] # locfitbw[9]$dp[6]
  myaicval[i] <- aicbinfun(y,n,preds,df[i])$aicbin
  loglik[i] <- aicbinfun(y,n,preds,df[i])$loglik
}
par(mfrow=c(1,2))
plot(myaicval~hval)
plot(df~hval)
hopt <- hval[which.min(myaicval)]
dfopt <- df[which.min(myaicval)]
cat("Minimizing values h and df =",hopt,dfopt,"\n") # df=4.1
#
# Now obtain the fit with the minimizing value of h
#
locgcv <- locfit(y~lp(x,deg=1,nn=0,h=hopt),data=bw,family="binomial")
xseq <- seq(min(x),max(x),.01)
preds <- predict(locgcv,newdata=xseq)
newx <- xseq*sdbw + meanbw
#
# Penalized regression splines for the BPD data
#
library(mgcv)
#attach(bw)
#
gammod <- gam(bw[,2]~s(x=bw$birthweight,k=10,fx=F,bs="cr",m=2),family=binomial)
orderbw <- cbind(BPD,birthweight)
orderbw <- orderbw[order(orderbw[,2]),]
orderfit <- cbind(birthweight,gammod$fitted)
orderfit <- orderfit[order(orderfit[,1]),]
#
# Fig 11.16
#
pdf("BPDcubic.pdf",width=4.5,height=4.5)
par(mfrow=c(1,1))
plot(birthweight,BPD,pch="|",xlab="Birthweight (grams)")
#plot(birthweight[1:50],BPD[1:50],pch="|",xlab="Birthweight (grams)")
lines(orderfit[,1],orderfit[,2],lty=1)
lrmod1 <- glm(BPD~birthweight,family=binomial)
xseq2 <- seq(min(birthweight),max(birthweight), 0.01)
#myexpit <- function(x,b0,b1){exp(b0+b1*x)/( 1+exp(b0+b1*x) )}
myexpit <- function(x,b0,b1){expit <- exp(b0+b1*x)/( 1+exp(b0+b1*x) )}
lines(xseq2,myexpit(xseq2,b0=lrmod1$coeff[1],b1=lrmod1$coeff[2]),lty=3)
lines(newx,preds,lty=2)
legend(x=0.3,y=0.75,legend=c("Cubic Spline","Local Likelihood","Linear Logistic"),
       bty="n",lty=1:3)
dev.off()
