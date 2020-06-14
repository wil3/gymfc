import tensorflow as tf
import os.path
import time


class CheckpointMonitor:
    """Helper class to monitor the Tensorflow checkpoints and call a callback
    when a new checkpoint has been created."""

    def __init__(self, checkpoint_dir, callback):
        """
        Args:
                checkpoint_dir: Directory to monitor where new checkpoint
                directories will be created
                callback: A callback for when a new checkpoint is created.
        """
        self.checkpoint_dir = checkpoint_dir
        self.callback = callback
        # Track which checkpoints have already been called. 
        self.processed = []

        self.watching = True

    def _check_new_checkpoint(self):
        """Update the queue with newly found checkpoints.

        When a checkpoint directory is created a 'checkpoint' file is created
        containing a list of all the checkpoints. We can monitor this file to
        determine when new checkpoints have been created.
        """
        # TODO (wfk) check if there is a way to get a callback when a file has
        # changed. 

        ckpt = tf.train.get_checkpoint_state(self.checkpoint_dir)
        for path in ckpt.all_model_checkpoint_paths:
            checkpoint_filename = os.path.split(path)[-1]
            if tf.train.checkpoint_exists(path):
                # Make sure there is a checkpoint meta file before allowing it
                # to be processed
                meta_file = path + ".meta"
                if os.path.isfile(meta_file):
                    if (checkpoint_filename not in self.processed):
                        self.callback(path)
                        self.processed.append(checkpoint_filename)
                else:
                    print ("Meta file {} doesn't exist.".format(meta_file))

    def start(self):

        # Sit and wait until the checkpoint directory is created, otherwise we
        # can't monitor it. If it never gets created this could be an indicator
        # something is wrong with the trainer.
        c=0
        while not os.path.isdir(self.checkpoint_dir):
            print("[WARN {}] Directory {} doesn't exist yet, waiting until "
                  "created...".format(c, self.checkpoint_dir))
            time.sleep(30)
            c+=1

        while self.watching:
            self._check_new_checkpoint()
            time.sleep(10)

