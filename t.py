#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
プラレール記述言語プロトタイプ

文法にはこだわらない(最終的にガルバンゾに移植する？)
「言語」がどういう動作をするとうれしいのかの議論のたたき台になるようなやつをつくる。
なおこのドキュメントは実行可能なテストケースでもある。

コンポーネントは1つ以上の継ぎ目(Joint)を持つ

>>> c1 = Straight()
>>> c1.joints
[Joint((0.00, 0.00, 0.00), 4, female, Joint((1.00, 0.00, 0.00), 0, male]

末端を表現するコンポーネント EndPointがある

>>> e = EndPoint((10, 0, 0), 0, True)

コンポーネントにコンポーネントをつなぐと新しいコンポーネントになる

>>> c2 = connect(e, c1)
>>> c2.joints
[Joint((11.00, 0.00, 0.00), 0, male]

末端を決めて、そこに分岐しないレールをつないでいく過程では
つなぎ方が一通りしかないので記述が楽である。
(裏返してもつなげるコンポーネントは、裏返したものを別コンポーネントとすればよい)
文法的には c1 + c2 + ... でも connect(c1, c2, c3, ...)でもよい。
残念ながらこの結合演算は結合則を満たさないことに注意。

分岐がある場合など複数のJointが余っているComponentにconnectすると
つなぎ方にあいまいさが残るのでエラーになる。

>>> connect(Straight(), Straight())
Traceback (most recent call last):
...
RuntimeError: Ambiguous Connection

そういう場合には1つのJointを選ぶ演算(select_jointとかSelectedJointとか)を使う

>>> c1 = Straight()
>>> connect(SelectedJoint(c1, 0), Straight()).joints
[Joint((-1.00, 0.00, 0.00), 4, female]
>>> connect(SelectedJoint(c1, 1), Straight()).joints
[Joint((2.00, 0.00, 0.00), 0, male]

こうやって結合していったものは木構造になる

>>> tree = connect(SelectedJoint(c1, 0), Straight())
>>> tree
<__main__.ConnectedComponent object at ...>
>>> tree.children
[<__main__.SelectedJoint object at ...>, <__main__.Straight object at ...>]
>>> tree.children[0].children
[<__main__.Straight object at ...>]

FUTURE WORKS

最終的にループを閉じるときには、それ用の演算close_loopかなんかが、
Jointの位置と向きが正しいかをチェックしてずれていたら警告すればよい。
こうやってプラレールの構造が記述出来たら、そこから分岐を頂点とする無向グラフに変換できて
「2つの記述を与えて、おなじ構造であるかチェックする(リファクタでバグが入っていないことの検証)」や
「ある地点から走らせて特定地点(もしくは開始地点)に戻ってくるまでに通る経路の表示(シミュレーション実行)」ができるようになる。

SelectedJointがあればユーザが明示的にEndPointオブジェクトを作る必要はないかもなぁ。
最初に1つComponentを置いて、Jointを選んでそこから伸ばしていく

各階層ごとの2次元での可視化は(各Componentの2次元画像を用意してもらえれば)さほど難しくないかも。
あと、僕はプラレールの寸法などは全く詳しくないので今は適当になっている。
回転の最小角度って45度でいいんだっけ？
あと、長さの最小単位。部品データの製作者しかいじらないから最小単位を1にするよりは
実世界の長さ(センチとか)にした方がよいと思う。完成品のサイズを見積もりやすい。

Componentがchildrenに追加されるタイミングでそのComponentに親へのリンクを付けた方がいいかも。うっかり同じ部品を2回使ったりしないように。
2つJointのあるComponentの両側にSelectedJointを使うと今の実装では木が分かれてしまうから。
parentリンクがあれば、2回目のSelectedJointした段階で、すでに親がいることがわかる。
指定したJointがルートComponentに存在するならそれを使えばOK、ないならエラーになるべき。
"""

import numpy as np

class Component(object):
    def __init__(self):
        self.position = None
        self.rotation = None

class Joint(object):
    "レールの継ぎ目を表現するオブジェクト"
    def __init__(self, position, rotation, is_male):
        assert len(position) == 3  # X, Y, Z
        self.position = np.array(position, dtype=np.float)
        assert isinstance(rotation, int)
        self.rotation = rotation % 8
        self.is_male = bool(is_male)

    def __repr__(self):
        return "Joint(({:.2f}, {:.2f}, {:.2f}), {}, {}".format(
            self.position[0], self.position[1], self.position[2],
            self.rotation, "male" if self.is_male else "female")


class ConnectedComponent(Component):
    def __init__(self, position, rotation, children, joints):
        self.position = position
        self.rotation = rotation
        self.children = children
        self.joints = joints


class EndPoint(Component):
    def __init__(self, position, rotation, is_male):
        self.position = position
        self.rotation = rotation
        self.joints = [Joint(position, rotation, is_male)]


class Straight(Component):
    def __init__(self):
        self.joints = [
            Joint((0, 0, 0), 4, False),
            Joint((1, 0, 0), 0, True),
        ]


def connect(c1, c2):
    if len(c1.joints) == 1:
        return _connect(c1, c2)
    elif len(c2.joints) == 1:
        return _connect(c2, c1)
    raise RuntimeError("Ambiguous Connection")


def _connect(c1, c2):
    j1 = c1.joints[0]
    j2s = [j for j in c2.joints if j.is_male != j1.is_male]
    if len(j2s) == 1:
        j2 = j2s[0]
        # connect j1 and j2
        if (j1.rotation + j2.rotation) !=  4:
            # 回転が必要
            raise NotImplementedError
        dpos = j1.position - j2.position
        for j in c2.joints:
            j.position += dpos
        c = ConnectedComponent(
            c1.position,
            c1.rotation,
            [c1, c2],
            [j for j in c2.joints if j != j2]
        )
        return c
    raise RuntimeError("Ambiguous Connection")


class SelectedJoint(Component):
    def __init__(self, component, joint_index):
        self.children = [component]
        j = component.joints[joint_index]
        self.joints = [j]
        self.position = j.position
        self.rotation = 0


def _test():
    import doctest
    doctest.testmod(optionflags=doctest.ELLIPSIS)

if __name__ == "__main__":
    _test()
