package team3647.frc2024.auto;

import edu.wpi.first.math.geometry.Pose2d;
import team3647.frc2024.constants.FieldConstants;

public class Repathing {

    public PieceState state = PieceState.None;
    // make an enum and map of pieces
    // check for object for every path when x > certain amount
    // search map for state piece and interrupt -> next state if not
    // path to desired pose
    // figure out a way to chain paths again

    public enum PieceState {
        None {
            @Override
            public PieceState nextState() {
                return None;
            }

            @Override
            public Pose2d desiredPose() {
                return FieldConstants.kOrigin;
            }
        },
        GoingOne {
            @Override
            public PieceState nextState() {
                return GoingTwo;
            }

            @Override
            public Pose2d desiredPose() {
                return FieldConstants.kOrigin;
            }
        },
        GoingTwo {
            @Override
            public PieceState nextState() {
                return GoingThree;
            }

            @Override
            public Pose2d desiredPose() {
                return FieldConstants.kOrigin;
            }
        },
        GoingThree {
            @Override
            public PieceState nextState() {
                return GoingFour;
            }

            @Override
            public Pose2d desiredPose() {
                return FieldConstants.kOrigin;
            }
        },
        GoingFour {
            @Override
            public PieceState nextState() {
                return GoingFive;
            }

            @Override
            public Pose2d desiredPose() {
                return FieldConstants.kOrigin;
            }
        },
        GoingFive {
            @Override
            public PieceState nextState() {
                return GoingTwo;
            }

            @Override
            public Pose2d desiredPose() {
                return FieldConstants.kOrigin;
            }
        };

        public abstract PieceState nextState();

        public abstract Pose2d desiredPose();
    }
}
