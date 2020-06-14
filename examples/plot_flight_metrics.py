import argparse
import glob
import os.path
import numpy as np
import matplotlib.pyplot as plt
def get_immediate_subdirectories(a_dir):
    return [name for name in os.listdir(a_dir)
        if os.path.isdir(os.path.join(a_dir, name))]

def compute_checkpoint_evaluation_metrics(ckpt):
    es = []
    rs = []
    dac = []
    us = []
    for trial in glob.glob(ckpt + "/*.csv"):
        fdata = np.loadtxt(trial, delimiter=",")
        pqr = fdata[:, 11:14]
        pqr_sp = fdata[:, 14:17]
        e = np.abs(pqr - pqr_sp)
        es.append(e)
        rs.append(fdata[:, -1])
        ac = fdata[:, 7:11]
        dac.append(np.abs(np.diff(ac, axis=0)))

        u = fdata[:, 17:21]
        us.append(u)


    return [np.mean(es), np.mean(dac), np.mean(us), np.mean(rs)]

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Monitor and evaluate Tensorflow checkpoints.")
    parser.add_argument('eval_dir', 
                        help="Directory where evaluation logs are saved.")
    args = parser.parse_args()

    ckpts = glob.glob(args.eval_dir + "/*/")

    evaluation_metrics = []
    num_ckpts = 0
    ckpts.sort(key=os.path.getmtime)
    for ckpt in ckpts:
        metrics = compute_checkpoint_evaluation_metrics(ckpt)
        evaluation_metrics.append(metrics)
        num_ckpts += 1

    evaluation_metrics = np.array(evaluation_metrics)

    print ("Num=", num_ckpts)
    x = range(num_ckpts)

    labels = ["MAE", "Ave. u", "Ave. delta action", "Ave. reward"]
    f, ax = plt.subplots(len(labels), sharex=True, sharey=False)
    for i in range(evaluation_metrics.shape[1]):
        ax[i].plot(x, evaluation_metrics[:, i])
        ax[i].set_ylabel(labels[i])
        ax[i].grid()

    ax[1].hlines(0.12, 0, num_ckpts, label="idle")
    ax[1].legend()

    ax[-1].set_xlabel("Checkpoint")
    plt.show()
