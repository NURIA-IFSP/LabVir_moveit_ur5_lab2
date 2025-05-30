
# UR5 Pick and Place com Garra - Código em Python

Este documento explica o código em **Python** para realizar uma operação de **pick and place** (pegar e posicionar) utilizando o robô UR5 com uma garra Robotiq 2F-85. O código faz uso do ROS, MoveIt e da API `moveit_commander`.

---

## Bibliotecas Utilizadas

```python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf
import copy
```

- **moveit_commander:** Interface MoveIt para Python.
- **geometry_msgs:** Mensagens para poses e vetores.
- **tf:** Manipulação de quaternions e transformações.
- **rospy:** Interface ROS em Python.

---

## Inicialização do Nó ROS e MoveIt

```python
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_pick_and_place', anonymous=True)
```

- Inicializa o ROS e MoveIt para controle do robô.

---

## Criação das Interfaces

```python
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
```

- **robot:** Interface geral do robô.
- **scene:** Gerencia os objetos de colisão.
- **group:** Controla o braço ("manipulator").

---

## Controle da Garra

### Abrir Garra

```python
def open_gripper():
    rospy.loginfo("Abrindo garra")
    # Aqui você deve colocar o código para abrir a garra, como publicação em um tópico ou serviço.
```

### Fechar Garra

```python
def close_gripper():
    rospy.loginfo("Fechando garra")
    # Aqui você deve colocar o código para fechar a garra.
```

---

## Adicionando Objetos na Cena

```python
def add_collision_objects(scene):
    rospy.sleep(2)

    # Mesa 1
    table1_pose = geometry_msgs.msg.PoseStamped()
    table1_pose.header.frame_id = "base_link"
    table1_pose.pose.position.x = 0.5
    table1_pose.pose.position.y = 0
    table1_pose.pose.position.z = 0.1
    scene.add_box("table1", table1_pose, size=(0.2, 0.2, 0.2))

    # Mesa 2
    table2_pose = geometry_msgs.msg.PoseStamped()
    table2_pose.header.frame_id = "base_link"
    table2_pose.pose.position.x = 0
    table2_pose.pose.position.y = 0.5
    table2_pose.pose.position.z = 0.1
    scene.add_box("table2", table2_pose, size=(0.2, 0.2, 0.2))

    # Objeto
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.frame_id = "base_link"
    object_pose.pose.position.x = 0.5
    object_pose.pose.position.y = -0.14
    object_pose.pose.position.z = 0.2
    scene.add_box("object", object_pose, size=(0.04, 0.04, 0.04))

    rospy.sleep(2)
```

---

## Função de Pick (Pegar)

```python
def pick(group):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = 0.5
    pose_target.position.y = -0.14
    pose_target.position.z = 0.3

    quaternion = tf.transformations.quaternion_from_euler(-6.28, 1.57, 1.57)
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]

    group.set_pose_target(pose_target)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    open_gripper()

    rospy.sleep(1)

    close_gripper()

    current_pose = group.get_current_pose().pose
    current_pose.position.z += 0.25
    group.set_pose_target(current_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
```

---

## Função de Place (Posicionar)

```python
def place(group):
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = 0
    pose_target.position.y = 0.5
    pose_target.position.z = 0.3

    quaternion = tf.transformations.quaternion_from_euler(0, 0, 1.57)
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]

    group.set_pose_target(pose_target)

    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(1)

    open_gripper()

    current_pose = group.get_current_pose().pose
    current_pose.position.x += 0.25
    group.set_pose_target(current_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
```

---

## Função Principal

```python
if __name__ == '__main__':
    try:
        add_collision_objects(scene)

        pick(group)
        rospy.sleep(1)

        place(group)
        rospy.sleep(1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
```

---

## 🔧 Observações Importantes

- **Garra:** No código, os comandos `open_gripper()` e `close_gripper()` são placeholders. Você deve implementar o controle da garra, normalmente via publicação em tópicos, serviço ou ação, dependendo da interface da garra Robotiq no seu setup.
- **Planejamento:** Pode ser refinado utilizando waypoints para aproximação e retirada mais suaves.
- **Colisão:** A cena é configurada para evitar colisões durante o movimento.

---

## 🚀 Funcionamento

1. Cria duas mesas (`table1` e `table2`) e um objeto (`object`) na cena virtual.
2. Move o braço até a posição do objeto sobre a `table1`.
3. Fecha a garra para pegar o objeto.
4. Leva o objeto até a `table2`.
5. Abre a garra para soltar.

---

## 🎯 Próximos Passos

- Implementar controle da garra com tópicos ROS, serviço ou ação.
- Melhorar movimentos com waypoints para evitar trajetórias lineares bruscas.
- Adicionar detecção de sucesso na pegada.

---
