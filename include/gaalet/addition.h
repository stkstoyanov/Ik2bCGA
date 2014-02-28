#ifndef __GAALET_SUMMATION
#define __GAALET_SUMMATION

namespace gaalet
{

template<class L, class R>
struct addition : public expression<addition<L, R>>
{
   typedef typename merge_lists<typename L::clist, typename R::clist>::clist clist;

   typedef typename metric_combination_traits<typename L::metric, typename R::metric>::metric metric;

   typedef typename element_type_combination_traits<typename L::element_t, typename R::element_t>::element_t element_t;

   addition(const L& l_ , const R& r_ )
      :  l(l_), r(r_)
   { }

   template<conf_t conf>
   element_t element() const {
      return l.element<conf>() + r.element<conf>();
   }

protected:
   const L& l;
   const R& r;
};

template<class L, class R>
struct subtraction : public expression<subtraction<L, R>>
{
   typedef typename merge_lists<typename L::clist, typename R::clist>::clist clist;

   typedef typename metric_combination_traits<typename L::metric, typename R::metric>::metric metric;

   typedef typename element_type_combination_traits<typename L::element_t, typename R::element_t>::element_t element_t;

   subtraction(const L& l_ , const R& r_ )
      :  l(l_), r(r_)
   { }

   template<conf_t conf>
   element_t element() const {
      return l.element<conf>() - r.element<conf>();
   }

protected:
   const L& l;
   const R& r;
};

} //end namespace gaalet

/// \brief Addition of two multivectors.
/// \ingroup ga_ops
template <class L, class R> inline
gaalet::addition<L, R>
operator+(const gaalet::expression<L>& l, const gaalet::expression<R>& r) {
   return gaalet::addition<L, R>(l, r);
}

/// \brief Subtraction of two multivectors.
/// \ingroup ga_ops
template <class L, class R> inline
gaalet::subtraction<L, R>
operator-(const gaalet::expression<L>& l, const gaalet::expression<R>& r) {
   return gaalet::subtraction<L, R>(l, r);
}

#endif
