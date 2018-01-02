using System.Numerics;
using System.Threading.Tasks;
using Windows.UI.Input.Spatial;
using Windows.Perception.Spatial;
using System.Diagnostics;

namespace CrazyflieClient
{
    //
    // Summary:
    //      Implementation of a flight controller for a UAP app.
    //      This implementation uses Spatial Input APIs to consume hand gestures (HoloLens)
    class GestureController : IFlightController
    {
        //
        // Summary:
        //      Objects for spatial interaction and localization
        private SpatialInteractionManager interactionManager;
        private SpatialGestureRecognizer gestureRecognizer;
        private SpatialLocator spatialLocator;
        private SpatialStationaryFrameOfReference stationaryFrameOfReference;

        //
        // Summary:
        //      Variable for storing the last observed gesture position offset
        private Vector3 lastGestureOffset;



        //
        // Summary:
        //      Variable for storing the last observed vertical position to maintain
        //      height between gestures.
        private float lastVerticalPosition;
        private const float maxVerticalPosition = 1.0f; // 1m
        private const float minVerticalPosition = 0.05f;// 5cm

        
        //
        // Summary:
        //      Scaling factor for gesture ranges (in meters)
        //      Gesture offsets are divided by this scalar to map a (-1,1) range to
        //      (-gestureRangeScale, gestureRangeScale)
        private const float gestureRangeScale = 0.25f;

        //
        // Summary:
        //      Boolean to keep track of arming state
        private bool isArmed = false;

        // 
        // Summary:
        //      Boolean to keep track of whether self leveling is on or off
        //      (Currently always on)
        private const bool isSelfLevelEnabled = true;
  
        public GestureController()
        {
            // Start out at the minimum vertical position
            lastVerticalPosition = minVerticalPosition;


            spatialLocator = SpatialLocator.GetDefault();

            gestureRecognizer = new SpatialGestureRecognizer(
                SpatialGestureSettings.Tap |
                SpatialGestureSettings.ManipulationTranslate);

            gestureRecognizer.ManipulationCanceled += OnManipulationCanceled;
            gestureRecognizer.ManipulationCompleted += OnManipulationCompleted;
            gestureRecognizer.ManipulationStarted += OnManipulationStarted;
            gestureRecognizer.ManipulationUpdated += OnManipulationUpdated;
            gestureRecognizer.Tapped += OnTapped;

            interactionManager = SpatialInteractionManager.GetForCurrentView();
            interactionManager.InteractionDetected += OnInteractionDetected;
        }

        // Summary:
        //      Helper to clear the state of the setpoints and tell the copter to disarm
        private void ClearSetpointsAndDisarm()
        {
            // Store the vertical position
            lastVerticalPosition = ComputeVerticalPosition();

            lastGestureOffset.X = 0;
            lastGestureOffset.Y = 0;
            lastGestureOffset.Z = 0;
            isArmed = false;
        }
        
        // Summary:
        //      Handler for interaction detection. Forwards interaction to the gesture recognizer.
        private void OnInteractionDetected(object sender, SpatialInteractionDetectedEventArgs e)
        {
            gestureRecognizer.CaptureInteraction(e.Interaction);
        }

        // Summary:
        //      Handler for manipulation started events. Obtains the stationary frame of reference for the gesture.
        private void OnManipulationStarted(object sender, SpatialManipulationStartedEventArgs e)
        {
            // Manipulation has started - obtain the frame of reference relative to when the gesture began
            stationaryFrameOfReference = spatialLocator.CreateStationaryFrameOfReferenceAtCurrentLocation();
        }

        // Summary:
        //      Handler for manipulation completed events. Clears setpoints and disarms.
        private void OnManipulationCompleted(object sender, SpatialManipulationCompletedEventArgs e)
        {
            ClearSetpointsAndDisarm();
        }

        // Summary:
        //      Handler for manipulation canceled events. Clears setpoints and disarms.
        private void OnManipulationCanceled(object sender, SpatialManipulationCanceledEventArgs e)
        {
            ClearSetpointsAndDisarm();
        }

        // Summary:
        //      Handler for manipulation updated events. Stores relative offsets for later processing.
        private void OnManipulationUpdated(object sender, SpatialManipulationUpdatedEventArgs e)
        {
            // Get the manipulation delta relative to the frame of reference from when the manipulation began
            // Using a stationary frame of reference prevents movements of the device from affecting the gesture offset
            SpatialManipulationDelta manipulationDelta = 
                e.TryGetCumulativeDelta(stationaryFrameOfReference.CoordinateSystem);

            // Store the offset
            lastGestureOffset = manipulationDelta.Translation;
        }

        // Summary:
        //      Handler for the Tap gesture
        //      Used to toggle the state of isArmed
        private void OnTapped(object sender, SpatialTappedEventArgs e)
        {
            isArmed = !isArmed;
        }

        private float ComputeVerticalPosition()
        {
            return Clamp(lastVerticalPosition + (lastGestureOffset.Y / gestureRangeScale),
                minVerticalPosition,
                maxVerticalPosition);
        }

        // Summary:
        //      Helper for clamping a value to a specified min and max
        private float Clamp(float value, float min, float max)
        {
            return (value < min) ? min : ((value > max) ? max : value);
        }

        //
        // IFlightController implementaiton
        //
        public async Task<FlightControlAxes> GetFlightControlAxes()
        {
            FlightControlAxes axes;

            // Populate axes and clamp to (-1,1) for RPY and (0,1) for T
            axes.roll = Clamp(lastGestureOffset.X / gestureRangeScale, -1, 1);
            axes.pitch = -1 * Clamp(lastGestureOffset.Z / gestureRangeScale, -1, 1); // Z is inverted
            axes.yaw = 0; // No yaw support


            // TEMP: Hard code thrust to be the absolute z position.
            // 
            // Z position should remain constant between gestures. Treat the current gesture offset as a delta
            // applied to the last known fixed z position. When a gesture is completed, update the fixed z delta.
            // The actual z position should be clamped within a fixed range, as well.

            axes.thrust = ComputeVerticalPosition();

            


            axes.isSelfLevelEnabled = isSelfLevelEnabled;
            axes.isArmed = isArmed;

            return axes;
        }
    }
}
