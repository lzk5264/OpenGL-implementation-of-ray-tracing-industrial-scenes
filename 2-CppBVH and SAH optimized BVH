> References
> 
> [EZRT](https://github.com/AKGWSB/EzRT/blob/main/part%202%20--%20BVH%20Accelerate%20Struct/tutorial%20(.md%20%26%20.pdf%20file)/tutorial.md)
# BVH

## Triangle Class

用于存储组成模型的三角形

```cpp
class Triangle    
{    
    Triangle(glm::vec3 P0, glm::vec3 P1, glm::vec3 P2) : p0(P0), p1(P1), p2(P2)    
    {    
        center = (p0 + p1 + p2) / 3.0f;    
    }    
public:
    glm::vec3 p0, p1, p2;    
    glm::vec3 center;       
};

```
center的作用是用于三角形的排序。cmp就是不同轴的比较函数。

假如我们已经将一个obj模型的顶点信息读入，存到了

`std::vector<Triangle> triangles`

## BVHNode Class

```cpp
class BVHNode    
{
public:
    BVHNode* left = NULL;    
    BVHNode* right = NULL;    
    int n, idx;    
    glm::vec3 AA, BB;      
};
```

n代表了该包围盒存储的三角形个数，因为我们只在递归到叶子节点时才会计算光线与三角形相交，所以n不等于0时代表了叶子节点。

idx是一个索引，triangles 数组中 `[index, index + n -1]` 范围的三角形都属于该节点

AA和BB分别代表了包围盒最左下角和最右上角的点的坐标。

## build BVH

```cpp
std::shared_ptr<BVHNode> BuildBVHNode(std::vector<Triangle> triangles, int l, int r, int n)
{

    auto node = std::make_shared<BVHNode>();

    //计算出大包围盒
    glm::vec3 min(std::numeric_limits<float>::max());
    glm::vec3 max(std::numeric_limits<float>::min());

    for (int i = l; i <= r; i++)
    {
        min = glm::min(min, glm::min(triangles[i].p0, glm::min(triangles[i].p1, triangles[i].p2)));
        max = glm::max(max, glm::max(triangles[i].p0, glm::max(triangles[i].p1, triangles[i].p2)));
    }

    node->AA = min;
    node->BB = max;
    //到达递归条件，直接返回
    if (r - l + 1 <= n)
    {
        node->n = r - l + 1;
        node->idx = l;
        return node;
    }   

    //要将包围盒分为两部分，我们需要选出分割轴
    glm::vec3 extent(max - min);

    int axis((extent.x > extent.y) ? (extent.x > extent.z ? 0 : 2) : (extent.y > extent.z ? 1 : 2));

    //确定分割的点

    std::sort(triangles.begin() + l, triangles.begin() + r + 1, [axis](const Triangle& A, const Triangle& B) { return A.center[axis] < B.center[axis]; });

    int mid((l + r) >> 1);

    //递归计算两个分割后的包围盒

    node->left = BuildBVHNode(triangles, l, mid, n);
    node->right = BuildBVHNode(triangles, mid + 1, r, n);

    //返回

    return node;
}
```

>目前没有验证方法，我就找GPT问了一下，似乎是没有问题。

# SAH
SAH（Surface Area Heuristic，表面积启发式）优化方法。

**为什么SAH?**
尽管简单的分割策略（如中点分割或最长轴分割）可以更快地构建BVH树，但这些方法可能会产生深度不平衡的树结构，导致某些遍历路径明显比其他路径更长。SAH通过评估不同分割方案的成本来帮助创建更加平衡的BVH树，提高了渲染效率。
复杂场景中的几何形状分布往往是不均匀的，简单的分割策略可能无法有效地处理这种不均匀性。SAH通过量化分割后的子树包围盒的表面积以及它们所包含的几何体数量，能够更好地适应场景的复杂性，实现高效的分割。

**什么是SAH?**
SAH通过计算分割后的两个子节点的表面积与包含几何体数量的比例，来预估分割的成本。具体地说，它评估通过每个可能的分割平面进行分割所产生的两个子包围盒的表面积，以及每个子包围盒中的几何体数量。理论上，表面积较小且包含的几何体数量较均衡的分割被认为是成本更低、效率更高的。

**怎么通过SAH找到更好的分割方法？**
在使用表面积启发式（SAH）评估BVH（Bounding Volume Hierarchy）树的分割方法时，代价的计算基于一种假设：遍历和相交测试的时间主要取决于遍历树节点的数量和进行几何相交测试的数量。SAH通过量化这些操作的预期成本来评估不同的分割策略。以下是如何使用SAH评估分割方法代价的详细步骤：

1. **定义成本模型**

首先，定义两个基本的成本参数：
- **遍历成本**$C_{trav}$：遍历一个内部节点所需的成本。
- **相交成本**$C_{intersect}$：检测射线与一个几何体（如三角形）相交所需的成本。

这些成本可以根据具体实现或平台进行调整。

2. **分割后的子集面积计算**

给定一个分割方案，将当前节点的几何体集合分成两个子集，分别对应左右子节点。对于每个子集，计算其轴对齐包围盒（AABB）的表面积。设左子节点的包围盒表面积为$A_{left}$，右子节点为$A_{right}$，而父节点（当前节点）的包围盒表面积为$A_{parent}$。

3. **计算分割后的期望成本**

使用下列公式来计算分割方案的期望成本：
$C_{split} = C_{trav} + \left( \frac{A_{left}}{A_{parent}} \cdot N_{left} + \frac{A_{right}}{A_{parent}} \cdot N_{right} \right) \cdot C_{intersect}$

其中，$N_{left}$和$N_{right}$分别是左右子集中几何体的数量。

这个公式的理解是：每次遍历需要支付一个固定的成本$C_{trav}$，加上进入每个子节点的概率（由其表面积与父节点表面积的比例表示）乘以该子节点内几何体数量的成本。

4. **遍历所有可能的分割方案**

为了找到最佳分割方案，需要遍历所有可能的分割位置（对于三角形网格，这通常是基于顶点位置或三角形中心点）。对每个可能的分割位置，使用上述步骤计算分割成本，并记录成本最低的分割方案。

5. **选择最佳分割**

最后，将所有可能的分割方案中成本最低的那个作为最终的分割策略。如果最佳的分割方案的成本高于不分割（即直接将所有几何体放在当前节点而形成叶节点）的成本，那么选择不分割的策略。不分割的成本可以简单地表示为：
$C_{leaf} = N_{parent} \cdot C_{intersect}$

其中，$N_{parent}$是父节点中几何体的数量。

使用SAH的关键在于，它能够在构建BVH树的过程中，平衡树的深度与节点中几何体数量的关系，从而优化渲染性能。通过精心选择分割方案，可以显著减少渲染时射线与几何体相交检测的计算量，尽管SAH的计算本身比较昂贵，但在许多高要求的渲染任务中，这种额外的预处理成本是值得的。
```cpp
std::shared_ptr<BVHNode> BuildBVHWithSAH(std::vector<Triangle> triangles, int l, int r, int n)
{
    auto node = std::make_shared<BVHNode>();

    //计算出大包围盒
    glm::vec3 min(std::numeric_limits<float>::max());
    glm::vec3 max(std::numeric_limits<float>::min());

    for (int i = l; i <= r; i++)
    {
        min = glm::min(min, glm::min(triangles[i].p0, glm::min(triangles[i].p1, triangles[i].p2)));
        max = glm::max(max, glm::max(triangles[i].p0, glm::max(triangles[i].p1, triangles[i].p2)));
    }

    node->AA = min;
    node->BB = max;
    //到达递归条件，直接返回
    if (r - l + 1 <= n)
    {
        node->n = r - l + 1;
        node->idx = l;
        return node;
    }

    //使用SAH寻找更好的分割方案
    // 提前记录未分割前的包围盒表面积
    glm::vec3 parentExtent(max - min);
    float parentS(2.0f * (parentExtent.x * parentExtent.y + parentExtent.y * parentExtent.z + parentExtent.z * parentExtent.x));

    float bestCost(std::numeric_limits<float>::max());
    int bestAxis;
    int bestSplit;
    //遍历xyz轴
    for (int axis = 0; axis < 3; axis++)
    {
        //对相应轴进行排序
        std::sort(triangles.begin() + l, triangles.begin() + r + 1, [axis](const Triangle& A, const Triangle& B) { return A.center[axis] < B.center[axis]; });
        //提前计算不同分割点形成的不同左右包围盒
        std::vector<glm::vec3> leftAA(r - l + 1, glm::vec3(std::numeric_limits<float>::max()));
        std::vector<glm::vec3> leftBB(r - l + 1, glm::vec3(std::numeric_limits<float>::min()));
        std::vector<glm::vec3> rightAA(r - l + 1, glm::vec3(std::numeric_limits<float>::max()));
        std::vector<glm::vec3> rightBB(r - l + 1, glm::vec3(std::numeric_limits<float>::min()));

        for (int i = l; i <= r; i++)
        {
            int bias((i == l) ? 0 : 1);
            leftAA[i - l] = glm::min(leftAA[i - l - bias], glm::min(triangles[i].p0, glm::min(triangles[i].p1, triangles[i].p2)));
            leftBB[i - l] = glm::max(leftBB[i - l - bias], glm::max(triangles[i].p0, glm::max(triangles[i].p1, triangles[i].p2)));
        }

        for (int i = r; i >= l; i--)
        {
            int bias((i == r) ? 0 : 1);
            rightAA[i - l] = glm::min(rightAA[i - l + bias], glm::min(triangles[i].p0, glm::min(triangles[i].p1, triangles[i].p2)));
            rightBB[i - l] = glm::max(rightBB[i - l + bias], glm::max(triangles[i].p0, glm::max(triangles[i].p1, triangles[i].p2)));
        }

        //遍历所有的分割方式
        for (int split = l; split < r; split++)
        {
            //计算这种分割方式形成的两个包围盒的表面积
            glm::vec3 leftExtent(leftBB[split] - leftAA[split]);
            glm::vec3 rightExtent(rightBB[split] - rightAA[split]);

            float leftS = 2.0f * (leftExtent.x * leftExtent.y + leftExtent.y * leftExtent.z + leftExtent.z * leftExtent.x);
            float rightS = 2.0f * (rightExtent.x * rightExtent.y + rightExtent.y * rightExtent.z + rightExtent.z * rightExtent.x);

            //计算这种分割方式的总代价
            float cost = (leftS * (split - l + 1) + rightS * (r - split)) / parentS;
            if (cost < bestCost)
            {
                bestCost = cost;
                bestAxis = axis;
                bestSplit = split;
            }
        }  
    }
    if (r - l + 1 < bestCost)
    {
        node->n = r - l + 1;
        node->idx = l;
        return node;
    }

    //找到最佳分割轴和分割点后，递归
    std::sort(triangles.begin() + l, triangles.begin() + r + 1, [bestAxis](const Triangle& A, const Triangle& B) { return A.center[bestAxis] < B.center[bestAxis]; });
    node->left = BuildBVHWithSAH(triangles, l, bestSplit, n);
    node->right = BuildBVHWithSAH(triangles, bestSplit + 1, r, n);

    return node;
}
```

# OpenGL 可视化

这两个函数比较抽象，无法直接验证其正确性，所以我选择使用OpenGL可视化包围盒来验证算法的正确性。
```cpp
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Headfile/Shader.h"
#include "Headfile/Camera.h"
#define STB_IMAGE_IMPLEMENTATION
#include "Headfile/model.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <numbers>

#include <iostream>



const unsigned int SCR_WIDTH = 1920;
const unsigned int SCR_HEIGHT = 1080;
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
//记录每一帧的鼠标位置
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
//是否第一次聚焦程序
bool firstMouse = true;
//记录每一帧的时间
float deltaTime = 0.0f;
float lastFrame = 0.0f;

std::vector<glm::vec3> vertices;     // 顶点坐标
std::vector<GLuint> indices;    // 顶点索引
std::vector<glm::vec3> lines;        // 线段端点坐标


void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}


void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);
}

void mouse_callback(GLFWwindow* window, double xPosIn, double yPosIn)
{
    float xPos = static_cast<float>(xPosIn), yPos = static_cast<float>(yPosIn);
    float xOffset, yOffset;
    if (firstMouse)
    {
        lastX = xPos;
        lastY = yPos;
        firstMouse = false;
    }
    xOffset = xPos - lastX;
    yOffset = lastY - yPos;
    camera.ProcessMouseMovement(xOffset, yOffset);
    lastX = xPos;
    lastY = yPos;
}

void scoll_callback(GLFWwindow* window, double xOffset, double yOffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yOffset));
}

unsigned int loadTexture(char const* path)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

unsigned int loadAlphaTexture(char const* path)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}


class Triangle
{
public:
    Triangle(glm::vec3 P0, glm::vec3 P1, glm::vec3 P2) : p0(P0), p1(P1), p2(P2)
    {
        center = (p0 + p1 + p2) / 3.0f;
    }

public:
    glm::vec3 p0, p1, p2;
    glm::vec3 center;

};

std::vector<Triangle> triangles;

void AddTriangle(std::shared_ptr<Triangle> triangle)
{
    if (triangle)
    {
        lines.push_back(triangle->p0 - glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p1 - glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p1 - glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p2 - glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p2 - glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p0 - glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p0 + glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p1 + glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p1 + glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p2 + glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p2 + glm::vec3(0.0005, 0.0005, 0.0005));
        lines.push_back(triangle->p0 + glm::vec3(0.0005, 0.0005, 0.0005));
    }
}

//读取OBJ文件到triangles中
// 读取 obj
void readObj(std::string filepath, std::vector<glm::vec3>& vertices, std::vector<GLuint>& indices) {
    // 打开文件流
    std::ifstream fin(filepath);
    std::string line;
    if (!fin.is_open()) {
        std::cout << "文件 " << filepath << " 打开失败" << std::endl;
        exit(-1);
    }

    // 增量读取
    int offset = vertices.size();

    // 按行读取
    while (std::getline(fin, line)) {
        std::istringstream sin(line);   // 以一行的数据作为 string stream 解析并且读取
        std::string type;
        GLfloat x, y, z;
        int v0, v1, v2;

        // 读取obj文件
        sin >> type;
        if (type == "v") {
            sin >> x >> y >> z;
            vertices.push_back(glm::vec3(x, y, z));
        }
        if (type == "f") {
            sin >> v0 >> v1 >> v2;
            indices.push_back(v0 - 1 + offset);
            indices.push_back(v1 - 1 + offset);
            indices.push_back(v2 - 1 + offset);
        }
    }
}

void AddLine(glm::vec3 p1, glm::vec3 p2)
{
    lines.push_back(p1);
    lines.push_back(p2);
}


class BVHNode
{
public:
    std::shared_ptr<BVHNode> left = NULL;
    std::shared_ptr<BVHNode> right = NULL;
    int n = 0, idx;
    glm::vec3 AA, BB;
};

void AddBox(std::shared_ptr<BVHNode> root)
{
    float x1 = root->AA.x, y1 = root->AA.y, z1 = root->AA.z;
    float x2 = root->BB.x, y2 = root->BB.y, z2 = root->BB.z;

    lines.push_back(glm::vec3(x1, y1, z1)), lines.push_back(glm::vec3(x2, y1, z1));
    lines.push_back(glm::vec3(x1, y1, z1)), lines.push_back(glm::vec3(x1, y1, z2));
    lines.push_back(glm::vec3(x1, y1, z1)), lines.push_back(glm::vec3(x1, y2, z1));
    lines.push_back(glm::vec3(x2, y1, z1)), lines.push_back(glm::vec3(x2, y1, z2));
    lines.push_back(glm::vec3(x2, y1, z1)), lines.push_back(glm::vec3(x2, y2, z1));
    lines.push_back(glm::vec3(x1, y2, z1)), lines.push_back(glm::vec3(x2, y2, z1));
    lines.push_back(glm::vec3(x1, y1, z2)), lines.push_back(glm::vec3(x1, y2, z2));
    lines.push_back(glm::vec3(x1, y2, z1)), lines.push_back(glm::vec3(x1, y2, z2));
    lines.push_back(glm::vec3(x1, y2, z2)), lines.push_back(glm::vec3(x2, y2, z2));
    lines.push_back(glm::vec3(x1, y1, z2)), lines.push_back(glm::vec3(x2, y1, z2));
    lines.push_back(glm::vec3(x2, y2, z1)), lines.push_back(glm::vec3(x2, y2, z2));
    lines.push_back(glm::vec3(x2, y1, z2)), lines.push_back(glm::vec3(x2, y2, z2));
}

std::shared_ptr<BVHNode> BuildBVH(std::vector<Triangle> triangles, int l, int r, int n)
{

    auto node = std::make_shared<BVHNode>();

    //计算出大包围盒
    glm::vec3 min(std::numeric_limits<float>::max());
    glm::vec3 max(std::numeric_limits<float>::min());

    for (int i = l; i <= r; i++)
    {
        min = glm::min(min, glm::min(triangles[i].p0, glm::min(triangles[i].p1, triangles[i].p2)));
        max = glm::max(max, glm::max(triangles[i].p0, glm::max(triangles[i].p1, triangles[i].p2)));
    }

    node->AA = min;
    node->BB = max;
    //到达递归条件，直接返回
    if (r - l + 1 <= n)
    {
        node->n = r - l + 1;
        node->idx = l;
        return node;
    }   

    //要将包围盒分为两部分，我们需要选出分割轴
    glm::vec3 extent(max - min);

    int axis((extent.x > extent.y) ? (extent.x > extent.z ? 0 : 2) : (extent.y > extent.z ? 1 : 2));

    //确定分割的点

    std::sort(triangles.begin() + l, triangles.begin() + r + 1, [axis](const Triangle& A, const Triangle& B) { return A.center[axis] < B.center[axis]; });

    int mid((l + r) >> 1);

    //递归计算两个分割后的包围盒

    node->left = BuildBVH(triangles, l, mid, n);
    node->right = BuildBVH(triangles, mid + 1, r, n);

    //返回

    return node;
}

std::shared_ptr<BVHNode> BuildBVHWithSAH(std::vector<Triangle> triangles, int l, int r, int n)
{
    auto node = std::make_shared<BVHNode>();

    //计算出大包围盒
    glm::vec3 min(std::numeric_limits<float>::max());
    glm::vec3 max(std::numeric_limits<float>::min());

    for (int i = l; i <= r; i++)
    {
        min = glm::min(min, glm::min(triangles[i].p0, glm::min(triangles[i].p1, triangles[i].p2)));
        max = glm::max(max, glm::max(triangles[i].p0, glm::max(triangles[i].p1, triangles[i].p2)));
    }

    node->AA = min;
    node->BB = max;
    //到达递归条件，直接返回
    if (r - l + 1 <= n)
    {
        node->n = r - l + 1;
        node->idx = l;
        return node;
    }

    //使用SAH寻找更好的分割方案
    // 提前记录未分割前的包围盒表面积
    glm::vec3 parentExtent(node->BB - node->AA);
    float parentS(2.0f * (parentExtent.x * parentExtent.y + parentExtent.y * parentExtent.z + parentExtent.z * parentExtent.x));

    float bestCost = std::numeric_limits<float>::max();
    int bestAxis = 0;
    int bestSplit = l;
    //遍历xyz轴
    for (int axis = 0; axis < 3; axis++)
    {
        //对相应轴进行排序
        std::sort(triangles.begin() + l, triangles.begin() + r + 1, [axis](const Triangle& A, const Triangle& B) { return A.center[axis] < B.center[axis]; });
        //提前计算不同分割点形成的不同左右包围盒
        std::vector<glm::vec3> leftAA(r - l + 1, glm::vec3(std::numeric_limits<float>::max()));
        std::vector<glm::vec3> leftBB(r - l + 1, glm::vec3(std::numeric_limits<float>::min()));
        std::vector<glm::vec3> rightAA(r - l + 1, glm::vec3(std::numeric_limits<float>::max()));
        std::vector<glm::vec3> rightBB(r - l + 1, glm::vec3(std::numeric_limits<float>::min()));

        for (int i = l; i <= r; i++)
        {
            int bias((i == l) ? 0 : 1);
            leftAA[i - l] = glm::min(leftAA[i - l - bias], glm::min(triangles[i].p0, glm::min(triangles[i].p1, triangles[i].p2)));
            leftBB[i - l] = glm::max(leftBB[i - l - bias], glm::max(triangles[i].p0, glm::max(triangles[i].p1, triangles[i].p2)));
        }

        for (int i = r; i >= l; i--)
        {
            int bias((i == r) ? 0 : 1);
            rightAA[i - l] = glm::min(rightAA[i - l + bias], glm::min(triangles[i].p0, glm::min(triangles[i].p1, triangles[i].p2)));
            rightBB[i - l] = glm::max(rightBB[i - l + bias], glm::max(triangles[i].p0, glm::max(triangles[i].p1, triangles[i].p2)));
        }
        
        //遍历所有的分割方式
        for (int split = l; split < r; split++)
        {
            //计算这种分割方式形成的两个包围盒的表面积
            glm::vec3 leftExtent(leftBB[split - l] - leftAA[split - l]);
            glm::vec3 rightExtent(rightBB[split - l] - rightAA[split - l]);

            float leftS = 2.0f * (leftExtent.x * leftExtent.y + leftExtent.y * leftExtent.z + leftExtent.z * leftExtent.x);
            float rightS = 2.0f * (rightExtent.x * rightExtent.y + rightExtent.y * rightExtent.z + rightExtent.z * rightExtent.x);

            //计算这种分割方式的总代价
            float cost = (leftS * (split - l + 1) + rightS * (r - split)) / parentS;
            if (cost < bestCost)
            {
                bestCost = cost;
                bestAxis = axis;
                bestSplit = split;
            }
        }
    }

    if (r - l + 1 < bestCost)
    {
        node->n = r - l + 1;
        node->idx = l;
        return node;
    }


    //找到最佳分割轴和分割点后，递归
    std::sort(triangles.begin() + l, triangles.begin() + r + 1, [bestAxis](const Triangle& A, const Triangle& B) { return A.center[bestAxis] < B.center[bestAxis]; });
    node->left = BuildBVHWithSAH(triangles, l, bestSplit, n);
    node->right = BuildBVHWithSAH(triangles, bestSplit + 1, r, n);

    return node;
}

void DfsNLevel(std::shared_ptr<BVHNode> root, int depth, int targetDepth)
{
    if (root == NULL) return;
    if (targetDepth == depth)
    {
        AddBox(root);
        return;
    }
    DfsNLevel(root->left, depth + 1, targetDepth);
    DfsNLevel(root->right, depth + 1, targetDepth);
}

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    //--------------------------------------------------------------------------

    GLFWwindow* window = glfwCreateWindow(1920, 1028, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scoll_callback);


    //------------------------------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    //-----------------------------------------------------------------------------------
    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    //-----------------------------------------------------------------------
    readObj("Rsrc/Models/Bunny/bunny.obj", vertices, indices);
    for (int i = 0; i < indices.size(); i += 3) {
        triangles.push_back(Triangle(vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]]));
    }
    std::shared_ptr<BVHNode> root(BuildBVHWithSAH(triangles, 0, triangles.size() - 1, 8));
    DfsNLevel(root, 0, 5);   // 可视化第 n 层 bvh

    //-------------------------------------------------------------
    Shader ourShader("shader/shadervs.glsl", "shader/shaderfs.glsl");
    //--------------------------------------------------------------------
    //VBO
    unsigned int VBO;
    glGenBuffers(1, &VBO);
    //VAO
    unsigned int VAO;
    glGenVertexArrays(1, &VAO);
    //EBO
    unsigned int EBO;
    glGenBuffers(1, &EBO);
    
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, (vertices.size() + lines.size()) * sizeof(glm::vec3), NULL, GL_STATIC_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec3) * vertices.size(), vertices.data());
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * vertices.size(), sizeof(glm::vec3) * lines.size(), lines.data());

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    //Rendering Loop
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        processInput(window);

        // render
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.setMat4("projection", projection);
        ourShader.setMat4("view", view);
        model = glm::translate(model, glm::vec3(0.0f, 0.0f, 2.0f));
        ourShader.setMat4("model", model);


        glBindVertexArray(VAO);
        ourShader.use();
        ourShader.setVec3("Color", 1.0f, 0.0f, 0.0f);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        ourShader.setVec3("Color", 1.0f, 1.0f, 1.0f);
        glDrawArrays(GL_LINES, vertices.size(), lines.size());


        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    //clear rsrc and terminate
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glfwTerminate();

    return 0;
}
```

```cpp
	#version 330 core
	
	out vec4 FragColor;
	
	uniform vec3 Color;
	
	
	void main()
	{
		FragColor = vec4(Color, 1.0);
	}

```
```cpp
#version 330 core
layout (location = 0) in vec3 aPos;


uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;


void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
```
有问题，BVH建树结果的第一层有很大重叠
![](https://raw.githubusercontent.com/lzk5264/OpenGL-implementation-of-ray-tracing-industrial-scenes/main/images/20240325182928.png)
但是第二层开始就没有这种问题了
![](https://raw.githubusercontent.com/lzk5264/OpenGL-implementation-of-ray-tracing-industrial-scenes/main/images/20240325182853.png)


