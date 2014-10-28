==============================================================
BMM : Bayesian Model Merging for Inverse Procedural Modeling
==============================================================

Source code of the paper

* Bayesian Grammar Learning for Inverse Procedural Modeling
   `Anđelo Martinović <http://homes.esat.kuleuven.be/~amartino/>`_, Luc Van Gool; CVPR 2013


**Abstract**
Within the fields of urban reconstruction and city modeling, shape grammars have emerged as a powerful tool for both synthesizing novel designs and reconstructing buildings. Traditionally, a human expert was required to write grammars for specific building styles, which limited the scope of method applicability. We present an approach to automatically learn two-dimensional attributed stochastic context-free grammars (2D-ASCFGs) from a set of labeled building facades. To this end, we use Bayesian Model Merging, a technique originally developed in the field of natural language processing, which we extend to the domain of two-dimensional languages. Given a set of labeled positive examples, we induce a grammar which can be sampled to create novel instances of the same building style. In addition, we demonstrate that our learned grammar can be used for parsing existing facade imagery. Experiments conducted on the dataset of Haussmannian buildings in Paris show that our parsing with learned grammars not only outperforms bottom-up classifiers but is also on par with approaches that use a manually designed style grammar.


*******
Setup
*******
THIS CODE IS PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE. Use at your own risk.

Recommendation: use QT creator to import the project.

*Requirements*: openCV version 2 (tested only with opencv 2.4). Create a symbolic link to the compiled openCV in the root folder and name it opencv2.4/

In the root folder of the repository, create three subfolders (or symbolic links): images/, labels/ and groundTruth/.


*labels/* : contains .txt files of labeled examples for grammar learning. Each image is represented as a width*height matrix of integers from 1.. (number of semantic classes), each number representing the class of the pixel.

*images/* : contains .jpg images that will be parsed with the learned grammar.

*groundTruth/* : contains .txt files in the same format as labels/, containing ground-truth labeling for grammar-based parsing.

The training-validation-evaluation split should be defined in testRig/fold{n}.txt.