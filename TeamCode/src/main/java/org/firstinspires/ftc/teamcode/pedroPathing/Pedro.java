package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces;
import org.firstinspires.ftc.teamcode.base.NonLinearActions;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.function.Function;

public abstract class Pedro {
    static{
        Constants.setConstants(FConstants.class, LConstants.class);
        PedroSleepUntilPose.setGetPose(()->{
            Pose pose=getPose();
            return new double[]{pose.getX(),pose.getY(),pose.getHeading()};
        });
    }
    public static Follower follower=new Follower(Components.getHardwareMap(), FConstants.class, LConstants.class);
    public static LambdaInterfaces.ReturningFunc<Pose> getPose = new Components.CachedReader<>(
            ()->{follower.updatePose(); return follower.getPose();},
            1
    )::cachedRead;
    public static Pose getPose(){
        return getPose.call();
    }
    public static void setStartingPose(Pose pose){
        follower.poseUpdater.setStartingPose(pose);
    }
    public static Follower getFollower(){
        return follower;
    }
    public static class PedroAction extends NonLinearActions.PathAction<PathChain>{
        private final boolean holdEnd;
        public PedroAction(Function<PathBuilder,PathBuilder> buildPath, boolean holdEnd) {
            super(()->buildPath.apply(follower.pathBuilder()).build());
            this.holdEnd=holdEnd;
        }
        public PedroAction(double x, double y, double heading, boolean holdEnd){ //Goes to the position indicated by the inputs
            this(
                    (PathBuilder b)-> b
                    .addBezierLine(new Point(follower.getPose()),new Point(new Pose(x,y,false)))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(),heading),
                    holdEnd
            );
        }
        public PedroAction(boolean holdEnd, double x, double y, double heading){ //Transforms from current position by the given inputs
            this(
                    (PathBuilder b)-> b
                            .addBezierLine(new Point(follower.getPose()),new Point(new Pose(follower.getPose().getX()+x,follower.getPose().getY()+y,false)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),follower.getPose().getHeading()+heading),
                    holdEnd
            );
        }
        @Override
        public boolean followPath(){
            if (isStart()){
                follower.followPath(getPath(),holdEnd);
            }
            follower.update();
            follower.updateCallbacks();
            return follower.isBusy();
        }
        @Override
        public void stopProcedure(){
            follower.breakFollowing();
        }
    }
    public static class PedroInstantAction extends NonLinearActions.InstantAction {
        public PedroInstantAction(Function<PathBuilder,PathBuilder> buildPath, boolean holdEnd) {
            super(()->follower.followPath(buildPath.apply(follower.pathBuilder()).build(),holdEnd));
        }
        public PedroInstantAction(double x, double y, double heading, boolean holdEnd) {
            this((PathBuilder b)-> b
                            .addBezierLine(new Point(follower.getPose()),new Point(x,y))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),heading),
                    holdEnd
            );
        }
        public PedroInstantAction(boolean holdEnd, double x, double y, double heading) {
            this((PathBuilder b)-> b
                            .addBezierLine(new Point(follower.getPose()),new Point(follower.getPose().getX()+x,follower.getPose().getY()+y))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),follower.getPose().getHeading()+heading),
                    holdEnd
            );
        }
    }
    public static class PedroSleepUntilPose extends NonLinearActions.SleepUntilPose{
        public PedroSleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance, double timeout) {
            super(x, y, heading, poseDistance, headingDistance, timeout);
        }
        public PedroSleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance) {
            super(x, y, heading, poseDistance, headingDistance);
        }
    }
}
