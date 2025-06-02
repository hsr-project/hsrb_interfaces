Overview
++++++++


提供機能
-----------------

PythonからHSRの各種機能を呼び出すための各種インターフェースを提供します。
また、インタラクティブな操作を提供するシェル機能も利用することができます。


ROS Interface
++++++++++++++

本パッケージはROSを介したロボットとの通信を行いますが、ユーザーがそれらを意識する必要がないよう設計されています。

実行時にROSノードに影響する環境変数( ``ROS_MASTER_URI`` 、``ROS_IP`` など）は、すべて同じように影響があります。


How to use
++++++++++

インタラクティブなシェルとして使う方法と、Pythonプログラムの一部として使う方法があります。

インタラクティブモードでは ``Ctrl-C`` (SIGINT)を入力することで、
実行中の動作（自律移動やアーム動作など）を途中でキャンセルすることができます。

ライブラリモードでは ``Ctrl-C`` を入力すると、通常のROSノード同様シャットダウンが行われます。

インタラクティブシェル
-------------------------

対話式にコマンドを実行させることができます。

.. sourcecode:: bash

    $ ihsrb

.. sourcecode:: python

    >>> HSR-B Interactive Shell 0.2.0
    >>>
    >>>       ____________  ______  _________       __  _______ ____
    >>>      /_  __/ __ \ \/ / __ \/_  __/   |     / / / / ___// __ \
    >>>       / / / / / /\  / / / / / / / /| |    / /_/ /\__ \/ /_/ /
    >>>      / / / /_/ / / / /_/ / / / / ___ |   / __  /___/ / _, _/
    >>>     /_/  \____/ /_/\____/ /_/ /_/  |_|  /_/ /_//____/_/ |_|
    >>>
    >>> In [1]: whole_body.move_to_go()
    >>>
    >>> In [2]:
    >>> Do you really want to exit ([y]/n)? y
    >>> Leaving HSR-B Interactive Shell

ライブラリとしての利用
-------------------------

ライブラリとして使う場合には、``hsrb_interface`` モジュールをインポートして下さい。

以下に腕関節を動かす簡単な例を記します。

.. sourcecode:: python

    import math
    import hsrb_interface

    with hsrb_interface.Robot() as robot:
        whole_body = robot.get('whole_body')
        goals = {
            'arm_lift_joint': 0.5,
            'arm_flex_joint': math.radians(-90)
        }
        whole_body.move_to_joint_positions(goals)

        whole_body.move_to_joint_positions(
            head_tilt_joint=math.radians(25)
        )

通信制御
~~~~~~~~~~

``hsrb_interface.Robot`` クラスのインスタンスを作成することでロボットとの通信を確立します。
全てのインスタンスで ``hsrb_interface.Robot.close()`` メソッドが呼び出されるか、
インスタンスが破壊されると通信は切断されます。

また、``with`` 文に対応しており、上記例のように ``close()`` 操作を自動化できます。

Actionベースのtf利用
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``hsrb_interface.Robot`` クラスのインスタンスを作成する際にuse\_tf\_serverのキーワード引数をTrueにすることで、
Actionベースのtfが使われます。



利用できる機能を調べる
~~~~~~~~~~~~~~~~~~~~~~

``hsrb_interface.Robot.list()`` メソッドを呼び出すことで、
現在利用できるリソースアイテム（移動台車、カメラ、関節制御グループなど）の一覧を得ることができます。

アイテムの取得
~~~~~~~~~~~~~~~~~~~~~~~

上記一覧で示された名前を指定して ``hsrb_interface.Robot.get()`` メソッドを呼び出すことで、
対応するアイテムを表すオブジェクトを取得できます。取得したオブジェクトに対し属性を設定したり、メソッドを呼び出すことで
ロボットの各種機能を利用することができます。

.. note:: 詳細はチュートリアルおよびAPIリファレンスをご覧ください。

Related Tutorials
-------------------

チュートリアルおよびリファレンスはHSR-Bマニュアルの関連する章をご覧ください。

チュートリアル
   7.HSRを開発する
APIリファレンス
   8.2.Python API
