# http://faculty.washington.edu/jonno/book/nonparametric2.R
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# BPD example in Section 11.5
#
logisticRegressionPlot <- function(trialDataFile, xName, yName, xLabel, yLabel) {
    library(locfit)
    myexpit <- function(x,b0,b1){expit <- exp(b0+b1*x)/( 1+exp(b0+b1*x) )}
    #bw <- read.table("birthweight.txt",header=T)
    bw <- read.table(trialDataFile,header=T)
    # bw <- bw[seq(1,100),]
    bw$kickSuccess <- bw$kickSuccess*(bw$numAlmostKicks==0)
    bw$timeout <- (1-bw$kickFailure) * (1-bw$collisionFailure) * (1-bw$kickSuccess)
    birthweight <- bw$birthweight <- data.matrix(bw[xName])
    BPD <- bw$BPD <- data.matrix(bw[yName])
    #birthweight <- bw$birthweight <- bw$chanceConstraint
    #BPD <- bw$BPD <- (1-bw$kickFailure) * (1-bw$collisionFailure) * (1-bw$kickSuccess)
    #BPD <- bw$BPD <- (1-kickFailure) * (1-collisionFailure) * (1-kickSuccess)
    #BPD <- bw$BPD <- bw$kickFailure
    #BPD <- bw$BPD <- bw$kickSuccess
    #BPD <- bw$BPD <- bw$collisionFailure
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
    clamp <- function(vals,minval,maxval) {
      minvals <- rep(minval,length(vals))
      maxvals <- rep(maxval,length(vals))
      result <- vals

      ind <- which(vals < minvals)
      result[ind] = minvals[ind]

      ind <- which(vals > maxvals)
      result[ind] = maxvals[ind]

      result
    }
    aicbinfun <- function(y,nl,p,df){
      probs <- clamp(p, 1e-6, 1e6)
      compls <- clamp(1 - p, 1e-6, 1e6)
      ndat <- length(y)
      dfterm <- 2*df
      # loglik <- 0
      # for (i in 1:ndat){
      #   logliki <- lchoose(nl[i],y[i])
      #            + y[i]*log(probs[i])
      #            + (nl[i]-y[i])*log(compls)
      #   loglik <- loglik + logliki
      # }
      loglik <- sum(lchoose(nl,y)+y*log(probs)+(nl-y)*log(compls))
      aicbin <- (-2*loglik  + dfterm)
      list(aicbin=aicbin,dfterm=dfterm,loglik=loglik)
    }
    n <- rep(1,length(y))
    hval <- seq(1,5,.1) # seq(1,5,.1)
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
    gammod <- gam(bw$BPD~s(x=bw$birthweight,k=10,fx=F,bs="cr",m=2),family=binomial)
    orderbw <- cbind(BPD,birthweight)
    orderbw <- orderbw[order(orderbw[,2]),]
    orderfit <- cbind(birthweight,gammod$fitted)
    orderfit <- orderfit[order(orderfit[,1]),]
    #
    # Fig 11.16
    #
    pdf(paste(trialDataFile,"-",yName,".pdf", sep=""),width=4.5,height=4.5)
    par(mfrow=c(1,1))
    plot(birthweight,BPD,pch="|",xlab=xLabel,ylab=yLabel)
    #plot(birthweight[1:200],BPD[1:200],pch="|",xlab="Birthweight (grams)")
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
}

logisticRegressionPlot("results-rrbt.tsv", "chanceConstraint", "kickSuccess", "Chance Constraint (%)", "Kick Success Rate (%)")
logisticRegressionPlot("results-rrbt.tsv", "chanceConstraint", "kickFailure", "Chance Constraint (%)", "Kick Failure Rate (%)")
logisticRegressionPlot("results-rrbt.tsv", "chanceConstraint", "collisionFailure", "Chance Constraint (%)", "Collision Failure Rate (%)")
logisticRegressionPlot("results-rrbt.tsv", "chanceConstraint", "timeout", "Chance Constraint (%)", "Timeout Failure Rate (%)")

# # Trial 1:
# logisticRegressionPlot("results-rrts.tsv", "ballObstacleOffsetFactor", "kickSuccess", "Obstacle Offset Factor", "Kick Success Rate (%)")
# logisticRegressionPlot("results-rrts.tsv", "ballObstacleOffsetFactor", "kickFailure", "Obstacle Offset Factor", "Kick Failure Rate (%)")
# logisticRegressionPlot("results-rrts.tsv", "ballObstacleOffsetFactor", "collisionFailure", "Obstacle Offset Factor", "Collision Failure Rate (%)")
# logisticRegressionPlot("results-rrts.tsv", "ballObstacleOffsetFactor", "timeout", "Obstacle Offset Factor", "Timeout Failure Rate (%)")

# Trial 2,3:
logisticRegressionPlot("results-rrts.tsv", "ballObstacleRadiusFactor", "kickSuccess", "Obstacle Radius Factor", "Kick Success Rate (%)")
logisticRegressionPlot("results-rrts.tsv", "ballObstacleRadiusFactor", "kickFailure", "Obstacle Radius Factor", "Kick Failure Rate (%)")
logisticRegressionPlot("results-rrts.tsv", "ballObstacleRadiusFactor", "collisionFailure", "Obstacle Radius Factor", "Collision Failure Rate (%)")
logisticRegressionPlot("results-rrts.tsv", "ballObstacleRadiusFactor", "timeout", "Obstacle Radius Factor", "Timeout Failure Rate (%)")
