package robot.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** A class that handles fetching vision frames and sending requests to a camera. */
public class CameraIO {
    protected final PhotonCamera cam;

    public CameraIO(String name) {
        this.cam = new PhotonCamera(name);
    }

    /** Sets the pipeline number for this camera. */
    public void setPipelineIndex(int pipeline) {
        cam.setPipelineIndex(pipeline);
    }

    /** Refreshes an instance of {@link CameraIO.RawData} with the latest frames. */
    public void refreshData(RawData inputs) {
        inputs.connected = cam.isConnected();
        inputs.results = cam.getAllUnreadResults();
    }

    /** Represents raw camera data from a photon camera every 0.02 seconds. */
    public static class RawData implements LoggableInputs {
        public boolean connected = true;
        public List<PhotonPipelineResult> results = new ArrayList<>();
        private final Packet serializer = new Packet(800);

        /** Fetches the best target from this data. */
        public Optional<PhotonTrackedTarget> bestTarget() {
            if (results.isEmpty() || !results.get(0).hasTargets()) {
                return Optional.empty();
            }
            return Optional.of(results.get(0).getBestTarget());
        }

        /** Pushes data to a log file. */
        @Override
        public void toLog(LogTable table) {
            for (int i = 0; i < results.size(); i++) {
                // pushes data from the result into the serializer.
                PhotonPipelineResult.photonStruct.pack(serializer, results.get(i));
                // then, logs the serialized data as a byte array (for replay, not human-readable).
                table.put("RawData/" + i, serializer.getWrittenDataCopy());
                serializer.clear();
            }
            table.put("Connected", connected);
            table.put("RawData/Total", results.size());
            table.put("SerializerSizeBytes", serializer.getSize()); // only for logging; not replayed
        }

        /** Overrides variables with data from a log file, effectively "replaying" the code. */
        @Override
        public void fromLog(LogTable table) {
            results = new ArrayList<>();
            connected = table.get("Connected", false);
            int numResults = table.get("RawData/Total", 0);
            for (int i = 0; i < numResults; i++) {
                serializer.setData(table.get("RawData/" + i, new byte[0]));
                results.add(PhotonPipelineResult.photonStruct.unpack(serializer));
                serializer.clear();
            }
        }
    }
}
