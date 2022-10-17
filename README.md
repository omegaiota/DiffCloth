# DiffCloth
Code repository for our paper [DiffCloth: Differentiable Cloth Simulation with Dry Frictional Contact](https://people.csail.mit.edu/liyifei/publication/diffcloth-differentiable-cloth-simulator/)
![](gifs/featured.png)

### 1. Download the repo:
**Make sure to use the `--recursive` option** to install the dependencies

`git clone --recursive https://github.com/omegaiota/DiffCloth.git`

### 2. Build CPP code:
From the top directory:
```
mkdir build
cd build
cmake ..
make
```
### 3. Optimize/Visualize Section 6 Experiments:

* Run optimization:
    ```
    ./DiffCloth -demo {demooptions} -mode optimize -seed {randseed}
    ```

    where `{demooptions}` is the name of the demos from the following options and `{randseed}` is an integer for random initialization of the initial guesses
    of the tasks.

    The corresponding option for each of the experiments is:
    * 6.1 T-shirt:  `tshirt`
    * 6.1 Sphere: `sphere`
    * 6.2 Hat: `hat`
    * 6.2 Sock: `sock`
    * 6.3 Dress: `dress`

* Visualize optimization iters:
    ```
    ./DiffCloth -demo {demooptions} -mode visualize -exp {expName}
    ```

    where `{expName}` is the iteration folder for visualization. The code repo comes with an example optimization run of T-shirt in `output/tshirt-exampleopt/`, and you can visualize the first iteration with

     ```
    ./DiffCloth -demo tshirt -mode visualize -exp tshirt-exampleopt/iter0/
    ```


The progress of the optimization is saved into the `output/` directory of the root folder. Intermediate progress are visualized using the custom written OpenGL viewer.

### 4. Build Python Binding and Run Hat Controller example:
Build Python Binding:

- Install anaconda for virtual environment.
- In project main folder `OmegaEngine`, run `python setup.py install` to install the python binding package. Rerun this command if you modify the CPP code.
- Create conda virtual environment: `conda env create python=3.8 --file environment.yml`, and activate it through `conda activate diffcloth`

Train/Test Hat Controller example:
- Navigate to `src/python_code`
* Test pretrained network: run `python hatController.py --eval --render --load_expname 20210809-trainedBest`
* Train network: run `python hatController.py --render`
* Resume train: run `python hatController.py --train_resume --load_expname [expName] --load_epoch [epochNum]`

Simulations are saved to `output/` directory of the root folder.  


### Citation

    @article{li2022diffcloth,
        author = {Li, Yifei and Du, Tao and Wu, Kui and Xu, Jie and Matusik, Wojciech},
        title = {DiffCloth: Differentiable Cloth Simulation with Dry Frictional Contact},
        year = {2022},
        publisher = {Association for Computing Machinery},
        address = {New York, NY, USA},
        issn = {0730-0301},
        url = {https://doi.org/10.1145/3527660},
        doi = {10.1145/3527660},
        abstract = {Cloth simulation has wide applications in computer animation, garment design, and robot-assisted dressing. This work presents a differentiable cloth simulator whose additional gradient information facilitates cloth-related applications. Our differentiable simulator extends a state-of-the-art cloth simulator based on Projective Dynamics (PD) and with dry frictional contact&nbsp;[Ly et&nbsp;al. 2020]},
        note = {Just Accepted},
        journal = {ACM Trans. Graph.},
        month = {mar},
        keywords = {cloth simulation, differentiable simulation, Projective Dynamics}
    }
