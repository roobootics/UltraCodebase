package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces;
import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.function.Function;

public abstract class Pedro {
    static{
        Constants.setConstants(FConstants.class, LConstants.class);
        PedroSleepUntilPose.setGetPose(()->{
            Pose pose=getPose();
            return new double[]{pose.getX(),pose.getY(),pose.getHeading()};
        });
    }
    public static final Follower follower=new Follower(Components.getHardwareMap(), FConstants.class, LConstants.class);
    private final static LambdaInterfaces.ReturningFunc<Pose> getPose = new Components.CachedReader<>(
            ()->{follower.updatePose(); return follower.getPose();},
            1
    )::cachedRead;

    public static Pose getPose(){
        return getPose.call();
    }
    public static void setStartingPose(Pose pose){
        follower.poseUpdater.setStartingPose(pose);
    }
    public static class PedroCommand extends Commands.PathCommand<PathChain>{
        private final boolean holdEnd;
        public PedroCommand(Function<PathBuilder,PathBuilder> buildPath, boolean holdEnd) {
            super(()->buildPath.apply(follower.pathBuilder()).build());
            this.holdEnd=holdEnd;
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
    public static class PedroLinearCommand extends PedroCommand{
        public PedroLinearCommand(double x, double y, double heading, boolean holdEnd){ //Goes to the position indicated by the inputs
            super(
                    (PathBuilder b)-> b
                            .addBezierLine(new Point(follower.getPose()),new Point(x,y))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),heading),
                    holdEnd
            );
        }
    }
    public static class PedroLinearChainCommand extends PedroCommand{
        public PedroLinearChainCommand(boolean holdEnd, Pose...poses){ //In a path chain, goes to all positions provided one at a time
            super(
                    (PathBuilder b)->{
                            ArrayList<Pose> poseList = new ArrayList<>();
                            poseList.add(follower.getPose());
                            Collections.addAll(poseList,poses);
                            for (int i=0;i<poses.length;i++){
                                b.addBezierLine(new Point(poseList.get(i)),new Point(poseList.get(i+1)));
                                b.setLinearHeadingInterpolation(poseList.get(i).getHeading(),poseList.get(i+1).getHeading());
                            }
                            return b;
                    },
                    holdEnd
            );
        }
    }
    public static class PedroLinearTransformCommand extends PedroCommand{
        public PedroLinearTransformCommand(double x, double y, double heading, boolean holdEnd){ //Transforms from current position by the given inputs
            super(
                    (PathBuilder b)-> b
                            .addBezierLine(new Point(follower.getPose()),new Point(follower.getPose().getX()+x,follower.getPose().getY()+y))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),follower.getPose().getHeading()+heading),
                    holdEnd
            );
        }
    }
    public static class PedroInstantCommand extends Commands.InstantCommand { //This action ends instantly and does not wait for the follower to reach its goal
        public PedroInstantCommand(Function<PathBuilder,PathBuilder> buildPath, boolean holdEnd) {
            super(()->follower.followPath(buildPath.apply(follower.pathBuilder()).build(),holdEnd));
        }
    }
    public static class PedroInstantLinearCommand extends PedroInstantCommand{
        public PedroInstantLinearCommand(double x, double y, double heading, boolean holdEnd) {
            super((PathBuilder b)-> b
                            .addBezierLine(new Point(follower.getPose()),new Point(x,y))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),heading),
                    holdEnd
            );
        }
    }
    public static class PedroInstantLinearChainCommand extends PedroInstantCommand{
        public PedroInstantLinearChainCommand(boolean holdEnd, Pose...poses){ //In a path chain, goes to all positions provided one at a time
            super(
                    (PathBuilder b)->{
                        ArrayList<Pose> poseList = new ArrayList<>();
                        poseList.add(follower.getPose());
                        Collections.addAll(poseList,poses);
                        for (int i=0;i<poses.length;i++){
                            b.addBezierLine(new Point(poseList.get(i)),new Point(poseList.get(i+1)));
                            b.setLinearHeadingInterpolation(poseList.get(i).getHeading(),poseList.get(i+1).getHeading());
                        }
                        return b;
                    },
                    holdEnd
            );
        }
    }
    public static class PedroInstantLinearTransformCommand extends PedroInstantCommand{
        public PedroInstantLinearTransformCommand(boolean holdEnd, double x, double y, double heading) {
            super((PathBuilder b)-> b
                            .addBezierLine(new Point(follower.getPose()),new Point(follower.getPose().getX()+x,follower.getPose().getY()+y))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(),follower.getPose().getHeading()+heading),
                    holdEnd
            );
        }
    }
    public static class PedroSleepUntilPose extends Commands.SleepUntilPose{
        public PedroSleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance, double timeout) {
            super(x, y, heading, poseDistance, headingDistance, timeout);
        }
        public PedroSleepUntilPose(double x, double y, double heading, double poseDistance, double headingDistance) {
            super(x, y, heading, poseDistance, headingDistance);
        }
    }
}
