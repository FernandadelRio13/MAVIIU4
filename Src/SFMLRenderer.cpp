#include "SFMLRenderer.h"

// Constructor de la clase SFMLRenderer
SFMLRenderer::SFMLRenderer(RenderWindow* window)
{
    wnd = window; // Asigna el puntero a la ventana de renderizado
}

// Destructor de la clase SFMLRenderer
SFMLRenderer::~SFMLRenderer(void)
{ }

// Dibuja un polígono sin relleno
void SFMLRenderer::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
    sf::ConvexShape polygon;
    polygon.setFillColor(Color(0, 0, 0, 0)); // Sin relleno
    polygon.setOutlineColor(box2d2SFMLColor(color)); // Color del borde

    polygon.setPointCount(vertexCount);
    for (int i = 0; i < vertexCount; ++i)
        polygon.setPoint(i, Vector2f(vertices[i].x, vertices[i].y)); // Establece los vértices del polígono

    wnd->draw(polygon); // Dibuja el polígono en la ventana
}

// Dibuja un polígono con relleno
void SFMLRenderer::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
    sf::ConvexShape polygon;
    polygon.setFillColor(box2d2SFMLColor(color)); // Establece el color de relleno
    polygon.setOutlineColor(box2d2SFMLColor(color)); // Color del borde

    polygon.setPointCount(vertexCount);
    for (int i = 0; i < vertexCount; ++i)
        polygon.setPoint(i, Vector2f(vertices[i].x, vertices[i].y)); // Establece los vértices del polígono

    wnd->draw(polygon); // Dibuja el polígono en la ventana
}

// Dibuja un círculo sin relleno
void SFMLRenderer::DrawCircle(const b2Vec2& center, float radius, const b2Color& color)
{
    sf::CircleShape circle(radius);
    circle.setOrigin(radius, radius);
    circle.setPosition(center.x, center.y);
    circle.setFillColor(Color(0, 0, 0, 0)); // Sin relleno
    circle.setOutlineColor(box2d2SFMLColor(color)); // Color del borde

    wnd->draw(circle); // Dibuja el círculo en la ventana
}

// Dibuja un círculo con relleno
void SFMLRenderer::DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color)
{
    sf::CircleShape circle(radius);
    circle.setOrigin(radius, radius);
    circle.setPosition(center.x, center.y);
    circle.setFillColor(box2d2SFMLColor(color)); // Establece el color de relleno
    circle.setOutlineColor(box2d2SFMLColor(color)); // Color del borde

    wnd->draw(circle); // Dibuja el círculo en la ventana
}

// Dibuja un segmento
void SFMLRenderer::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
{
    sf::Vertex line[] =
    {
        sf::Vertex(sf::Vector2f(p1.x, p1.y), box2d2SFMLColor(color)),
        sf::Vertex(sf::Vector2f(p2.x, p2.y), box2d2SFMLColor(color))
    };

    wnd->draw(line, 2, sf::Lines); // Dibuja el segmento en la ventana
}

// Dibuja una transformación
void SFMLRenderer::DrawTransform(const b2Transform& xf)
{
    // Dibuja los ejes de la transformación
    b2Vec2 p1 = xf.p, p2;
    const float k_axisScale = 20.0f;
    p2 = p1 + k_axisScale * xf.q.GetXAxis();

    sf::Vertex line[] =
    {
        sf::Vertex(sf::Vector2f(p1.x, p1.y), Color::Red),
        sf::Vertex(sf::Vector2f(p2.x, p2.y), Color::Green)
    };
    wnd->draw(line, 2, sf::Lines);

    p2 = p1 + k_axisScale * xf.q.GetYAxis();

    sf::Vertex line2[] =
    {
        sf::Vertex(sf::Vector2f(p1.x, p1.y), Color::Blue),
        sf::Vertex(sf::Vector2f(p2.x, p2.y), Color::Yellow)
    };
    wnd->draw(line2, 2, sf::Lines);
}

// Dibuja un punto
void SFMLRenderer::DrawPoint(const b2Vec2& p, float size, const b2Color& color)
{
    const float radius = 0.5f; // Radio del punto
    sf::CircleShape circle(radius);
    circle.setOrigin(radius, radius);
    circle.setPosition(p.x, p.y);
    circle.setFillColor(box2d2SFMLColor(color)); // Color del punto
    circle.setOutlineColor(box2d2SFMLColor(color)); // Color del borde

    wnd->draw(circle); // Dibuja el punto en la ventana
}

// Dibuja un texto
void SFMLRenderer::DrawString(int x, int y, const char* string, ...)
{
    sf::Text text;

    text.setString(string);
    text.setPosition(x, y);

    wnd->draw(text); // Dibuja el texto en la ventana
}

// Dibuja un AABB (Axis-Aligned Bounding Box)
void SFMLRenderer::DrawAABB(b2AABB* aabb, const b2Color& color)
{
    const auto halfSize = aabb->GetExtents();
    sf::RectangleShape rectangle(sf::Vector2f(halfSize.x, halfSize.y) * 2.0f);
    rectangle.setOrigin(halfSize.x, halfSize.y);
    const auto center = aabb->GetCenter();
    rectangle.setPosition(center.x, center.y);
    rectangle.setFillColor(Color(0, 0, 0, 0)); // Sin relleno
    rectangle.setOutlineColor(box2d2SFMLColor(color)); // Color del borde

    wnd->draw(rectangle); // Dibuja el AABB en la ventana
}

// Convierte un color de Box2D a un color de SFML
Color SFMLRenderer::box2d2SFMLColor(const b2Color& _color)
{
    Uint8 R, G, B;
    R = (Uint8)(_color.r * 255);
    G = (Uint8)(_color.g * 255);
    B = (Uint8)(_color.b * 255);

    return Color(R, G, B); // Devuelve el color en el formato de SFML
}