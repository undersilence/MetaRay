
template <class A2V, class V2F, class ColorFormat> class IShader {
protected:
  virtual V2F vertex_shader(const A2V &a2v) = 0;
  virtual ColorFormat fragment_shader(const V2F &v2f) = 0;
};

template <class A2V, class V2F, class OutFormat>
class SimpleShader : public IShader<A2V, V2F, OutFormat> {
public:
  SimpleShader() = default;
  virtual void test();

protected:
  virtual V2F vertex_shader(const A2V &a2v);
  virtual OutFormat fragment_shader(const V2F &v2f);
};