# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #

import json
import logging
from typing import Any, Dict, Optional, Sequence, Tuple, Union

logger = logging.getLogger(__name__)

_global_tracer: Union['Tracer', None] = None
_saved_global_tracer: Union['Tracer', None] = None
_root_context: Union['Context', None] = None


def global_tracer() -> Union['Tracer', None]:
    return _global_tracer


def root_context() -> Union['Context', None]:
    return _root_context


def disable_tracing() -> None:
    global _saved_global_tracer
    global _global_tracer
    if _global_tracer:
        _saved_global_tracer = _global_tracer
        _global_tracer = None


def enable_tracing() -> bool:
    global _global_tracer
    if _saved_global_tracer:
        _global_tracer = _saved_global_tracer
        return True
    return False


def tracing_enabled() -> bool:
    return _global_tracer is not None


from opentelemetry import trace, context
from opentelemetry.context import get_current as otel_get_current_context
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.sdk.resources import SERVICE_NAME, Resource
from opentelemetry.sdk.trace import Tracer, TracerProvider
from opentelemetry.sdk.trace.export import (
    BatchSpanProcessor,
)
from opentelemetry.sdk.trace.sampling import ParentBasedTraceIdRatio, ALWAYS_ON
from opentelemetry.trace.propagation import set_span_in_context as otel_set_span_in_context
from opentelemetry.trace.propagation.tracecontext import TraceContextTextMapPropagator


def telemetry_init(exporter: str, sample_rate: Optional[float] = 1.0) -> None:
    try:
        # Setup tracing

        if 0.0 < sample_rate < 1.0:
            # Sampler that respects its parent span's sampling decision, but otherwise samples
            # probabilistically based on `rate`.
            sampler = ParentBasedTraceIdRatio(sample_rate)
        else:
            # Get sampler from environment or default. Default is typically always-on sampler.
            sampler = ALWAYS_ON

        logger.info(f"penTelemetry: Attempting to connect to OLTP exporter at {exporter}")

        provider = TracerProvider(resource=Resource.create({SERVICE_NAME: "tibit-ponauto"}),
                                  sampler=sampler)
        trace.set_tracer_provider(provider)

        global _global_tracer
        _global_tracer = trace.get_tracer(__name__)
        logger.info(f"OpenTelemetry: Global tracer initialized. Sample Rate: {(sample_rate * 100):.1f}%")

        oltp_exporter = OTLPSpanExporter(endpoint=exporter)
        span_processor = BatchSpanProcessor(oltp_exporter)
        trace.get_tracer_provider().add_span_processor(span_processor)

        logger.info("*** OpenTelemetry: OLTP exporter and span processing enabled successfully")

        #
        # Save off a ROOT context that can be used to guarantee we start a new trace when required
        if _global_tracer:
            global _root_context
            _root_context = context.get_current()

    except Exception as e:
        logger.warn(f"OpenTelemetry: Initialization failed failed: {e}")


def add_trace_attributes(attributes: Dict[str, Union[str, bool, int, float,
Sequence[str], Sequence[bool],
Sequence[int], Sequence[float]]]) -> None:
    """ If tracing installed, add the attributes to the current span """
    if _global_tracer:
        span = trace.get_current_span()
        if span:
            span.set_attributes(attributes)


def add_trace_event(name: str,
                    attributes: Optional[Dict[str, Union[str, bool, int, float,
                    Sequence[str], Sequence[bool],
                    Sequence[int], Sequence[float]]]] = None,
                    timestamp: Optional[int] = None):
    """ If tracing installed, add the event to the current span """
    if _global_tracer:
        span = trace.get_current_span()
        if span:
            span.add_event(name, attributes=attributes, timestamp=timestamp)


def extract_span_context(span: 'trace_api.Span') -> Tuple[Dict[str, Any], bytes]:
    """
    Helper function to extract the current trace context and provide it as
    a dictionary and a binary string that can be attached to a message and sent
    to another thread/process.

    Actual attachment of the context to the message and the extraction is up
    to the caller
    """
    if span and _global_tracer:
        context = otel_set_span_in_context(span)
        carrier = {}
        TraceContextTextMapPropagator().inject(carrier, context=context)
        header = {"traceparent": carrier["traceparent"]}
        trace_context = bytes(json.dumps(header, separators=(",", ":")), "utf-8")
    else:
        header = {}
        trace_context = b""

    return header, trace_context


def set_span_in_context(span: 'Span', context: Optional['Context'] = None) -> 'Context':
    if not _global_tracer:
        return None
    return otel_set_span_in_context(span)


def restore_span_context(otel_context):
    if not _global_tracer:
        return None

    if isinstance(otel_context, dict):
        carrier = otel_context
    elif isinstance(otel_context, bytes):
        carrier = json.loads(otel_context.decode("utf-8"))
    else:
        # Allows it to be called with 'None' and start up a brand new trace span
        carrier = None

    return TraceContextTextMapPropagator().extract(carrier) if carrier else None


def get_current_span() -> 'trace_api.Span':
    """
    Get the current span

    If OpenTelemetry is not installed or enabled, this function will return 'None'

    If OpenTelemetry is installed and enabled, but there is not a current span, this
    will return the default 'null' or 'nop' trace span.
    """
    if not _global_tracer:
        return None

    return trace.get_current_span()


def get_current_span_context() -> 'Context':
    """
    Get the current span's Context

    If OpenTelemetry is not installed or enabled, this function will return 'None'
    """
    if not _global_tracer:
        return None

    return otel_get_current_context()
